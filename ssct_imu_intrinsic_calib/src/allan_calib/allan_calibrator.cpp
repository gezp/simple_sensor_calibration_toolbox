// Copyright 2026 Gezp (https://github.com/gezp).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ssct_imu_intrinsic_calib/allan_calib/allan_calibrator.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

#include <opencv2/imgproc.hpp>

#include "ssct_imu_intrinsic_calib/allan_calib/allan_plot.hpp"
#include "ssct_imu_intrinsic_calib/utils.hpp"

namespace ssct_imu_intrinsic_calib
{
namespace
{

struct AllanCurve
{
  std::vector<double> tau;
  std::vector<double> adev;
};

std::vector<int> cluster_sizes(int num_samples, int num_clusters)
{
  std::vector<int> sizes;
  if (num_samples < 3 || num_clusters < 1) {
    return sizes;
  }
  const int max_cluster = std::max(1, num_samples / 9);
  for (int i = 0; i < num_clusters; i++) {
    const double ratio = num_clusters == 1 ? 0.0 :
      static_cast<double>(i) / static_cast<double>(num_clusters - 1);
    const int cluster = static_cast<int>(
      std::llround(std::exp(ratio * std::log(static_cast<double>(max_cluster)))));
    const int value = std::clamp(cluster, 1, max_cluster);
    if (sizes.empty() || sizes.back() != value) {
      sizes.push_back(value);
    }
  }
  return sizes;
}

AllanCurve compute_curve(const Eigen::VectorXd & values, double dt, int num_clusters)
{
  AllanCurve curve;
  const int num_samples = static_cast<int>(values.size());
  Eigen::VectorXd prefix = Eigen::VectorXd::Zero(num_samples + 1);
  for (int i = 0; i < num_samples; i++) {
    prefix[i + 1] = prefix[i] + values[i];
  }
  for (const int cluster : cluster_sizes(num_samples, num_clusters)) {
    const int count = num_samples - 2 * cluster + 1;
    double sum = 0.0;
    for (int k = 0; k < count; k++) {
      const double first = prefix[k + cluster] - prefix[k];
      const double second = prefix[k + 2 * cluster] - prefix[k + cluster];
      const double diff = (second - first) / static_cast<double>(cluster);
      sum += diff * diff;
    }
    const double variance = sum / (2.0 * static_cast<double>(count));
    curve.tau.push_back(static_cast<double>(cluster) * dt);
    curve.adev.push_back(std::sqrt(std::max(0.0, variance)));
  }
  return curve;
}

double median(std::vector<double> values)
{
  const size_t middle = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + middle, values.end());
  return values[middle];
}

std::string format_vector(const Eigen::Vector3d & vector)
{
  std::ostringstream stream;
  stream << std::setprecision(9) << "[" << vector.x() << ", " << vector.y()
         << ", " << vector.z() << "]";
  return stream.str();
}

double estimate_noise_density(const AllanCurve & curve)
{
  std::vector<double> values;
  for (size_t i = 1; i < curve.tau.size(); i++) {
    if (curve.adev[i] <= 0.0 || curve.adev[i - 1] <= 0.0) {
      continue;
    }
    const double slope =
      (std::log(curve.adev[i]) - std::log(curve.adev[i - 1])) /
      (std::log(curve.tau[i]) - std::log(curve.tau[i - 1]));
    if (slope < -0.2) {
      values.push_back(curve.adev[i] * std::sqrt(curve.tau[i]));
    }
  }
  if (values.empty()) {
    const size_t end = std::max<size_t>(1, curve.tau.size() / 4);
    for (size_t i = 0; i < end; i++) {
      if (curve.adev[i] > 0.0) {
        values.push_back(curve.adev[i] * std::sqrt(curve.tau[i]));
      }
    }
  }
  return values.empty() ? 0.0 : median(values);
}

double estimate_random_walk(const AllanCurve & curve, bool & valid)
{
  std::vector<double> values;
  for (size_t i = 1; i < curve.tau.size(); i++) {
    if (curve.adev[i] <= 0.0 || curve.adev[i - 1] <= 0.0) {
      continue;
    }
    const double slope =
      (std::log(curve.adev[i]) - std::log(curve.adev[i - 1])) /
      (std::log(curve.tau[i]) - std::log(curve.tau[i - 1]));
    if (slope > 0.2) {
      values.push_back(curve.adev[i] * std::sqrt(3.0 / curve.tau[i]));
    }
  }
  valid = values.size() >= 2;
  return valid ? median(values) : 0.0;
}

Eigen::VectorXd axis_values(
  const std::vector<ssct_common::ImuData> & samples, int axis, bool gyro)
{
  Eigen::VectorXd values(samples.size());
  for (size_t i = 0; i < samples.size(); i++) {
    values[static_cast<int>(i)] = gyro ?
      samples[i].angular_velocity[axis] : samples[i].linear_acceleration[axis];
  }
  return values;
}

}  // namespace

AllanCalibrator::AllanCalibrator(const YAML::Node & config)
{
  if (!config) {
    return;
  }
  if (config["duration"]) {
    duration_ = config["duration"].as<double>();
  }
  if (config["plot_interval"]) {
    plot_interval_ = config["plot_interval"].as<double>();
  }
  if (config["num_clusters"]) {
    num_clusters_ = config["num_clusters"].as<int>();
  }
  if (config["min_samples"]) {
    min_samples_ = config["min_samples"].as<int>();
  }
}

cv::Mat AllanCalibrator::get_new_debug_image() const
{
  const cv::Mat bgr = plot_.draw();
  cv::Mat rgb;
  cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
  return rgb;
}

void AllanCalibrator::reset()
{
  samples_.clear();
  progress_ = 0;
  message_ = "start calibration: keep the IMU completely stationary.";
  accel_noise_density_.setZero();
  accel_random_walk_.setZero();
  gyro_noise_density_.setZero();
  gyro_random_walk_.setZero();
  accel_random_walk_valid_ = false;
  gyro_random_walk_valid_ = false;
  plot_.clear_data();
  last_update_duration_ = -1.0;
}

bool AllanCalibrator::add_data(
  const std::vector<ssct_common::ImuData> & samples)
{
  samples_.insert(samples_.end(), samples.begin(), samples.end());
  if (samples_.empty()) {
    message_ = "waiting for IMU data.";
    progress_ = 1;
    return false;
  }
  const double duration = samples_.back().time - samples_.front().time;
  progress_ = static_cast<uint8_t>(
    duration_ > 0.0 ? std::min(95.0, 95.0 * duration / duration_) : 95);
  std::ostringstream stream;
  stream << "keep the IMU stationary: " << duration << " / " << duration_
         << " s, " << samples_.size() << " samples";
  message_ = stream.str();
  if (duration >= duration_) {
    return false;
  }
  const bool first_update = last_update_duration_ < 0.0 && duration >= plot_interval_;
  const bool periodic_update = last_update_duration_ >= 0.0 && plot_interval_ > 0.0 &&
    duration - last_update_duration_ >= plot_interval_;
  if (plot_interval_ > 0.0 && (first_update || periodic_update)) {
    last_update_duration_ = duration;
    const bool updated = estimate_allan(samples_);
    if (updated) {
      message_ = "intermediate result (not saveable)\n" + message_;
    }
    return updated;
  }
  return false;
}

bool AllanCalibrator::is_ready() const
{
  return !samples_.empty() &&
         samples_.back().time - samples_.front().time >= duration_;
}

bool AllanCalibrator::optimize()
{
  progress_ = 100;
  return estimate_allan(samples_);
}

ssct_common::ImuIntrinsicParam AllanCalibrator::get_result() const
{
  ssct_common::ImuIntrinsicParam result;
  result.accel_matrix.setIdentity();
  result.accel_bias.setZero();
  result.accel_noise_density = accel_noise_density_;
  result.accel_random_walk = accel_random_walk_valid_ ?
    accel_random_walk_ : Eigen::Vector3d::Zero();
  result.gyro_matrix.setIdentity();
  result.gyro_bias.setZero();
  result.gyro_noise_density = gyro_noise_density_;
  result.gyro_random_walk = gyro_random_walk_valid_ ?
    gyro_random_walk_ : Eigen::Vector3d::Zero();
  return result;
}

bool AllanCalibrator::estimate_allan(
  const std::vector<ssct_common::ImuData> & samples)
{
  message_.clear();
  plot_.clear_data();
  if (static_cast<int>(samples.size()) < min_samples_) {
    std::ostringstream stream;
    stream << "not enough samples: " << samples.size() << " < " << min_samples_;
    message_ = stream.str();
    return false;
  }
  std::vector<double> deltas;
  deltas.reserve(samples.size());
  for (size_t i = 1; i < samples.size(); i++) {
    const double delta = samples[i].time - samples[i - 1].time;
    if (delta > 0.0) {
      deltas.push_back(delta);
    }
  }
  if (deltas.size() < static_cast<size_t>(min_samples_ - 1)) {
    message_ = "IMU timestamps are not strictly increasing";
    return false;
  }
  const size_t middle = deltas.size() / 2;
  std::nth_element(deltas.begin(), deltas.begin() + middle, deltas.end());
  const double dt = deltas[middle];
  if (!(dt > 0.0)) {
    message_ = "invalid IMU sample period";
    return false;
  }
  const double sample_rate = 1.0 / dt;

  for (int axis = 0; axis < 3; axis++) {
    const auto gyro_curve = compute_curve(
      axis_values(samples, axis, true), dt, num_clusters_);
    plot_.set_gyro_curve(axis, gyro_curve.tau, gyro_curve.adev);
    gyro_noise_density_[axis] = estimate_noise_density(gyro_curve);
    bool gyro_valid = false;
    gyro_random_walk_[axis] = estimate_random_walk(gyro_curve, gyro_valid);
    gyro_random_walk_valid_ =
      axis == 0 ? gyro_valid : (gyro_random_walk_valid_ && gyro_valid);

    const auto accel_curve = compute_curve(
      axis_values(samples, axis, false), dt, num_clusters_);
    plot_.set_accel_curve(axis, accel_curve.tau, accel_curve.adev);
    accel_noise_density_[axis] = estimate_noise_density(accel_curve);
    bool accel_valid = false;
    accel_random_walk_[axis] = estimate_random_walk(accel_curve, accel_valid);
    accel_random_walk_valid_ =
      axis == 0 ? accel_valid : (accel_random_walk_valid_ && accel_valid);
  }

  std::ostringstream stream;
  stream << "samples: " << samples.size() << "\n";
  stream << "duration: " << samples.back().time - samples.front().time << " s\n";
  stream << "sample_rate: " << sample_rate << " Hz\n";
  stream << "accel_noise_density: " << format_vector(accel_noise_density_) << "\n";
  stream << "accel_random_walk: ";
  if (accel_random_walk_valid_) {
    stream << format_vector(accel_random_walk_);
  } else {
    stream << "not observed (saved as zero)";
  }
  stream << "\ngyro_noise_density: " << format_vector(gyro_noise_density_) << "\n";
  stream << "gyro_random_walk: ";
  if (gyro_random_walk_valid_) {
    stream << format_vector(gyro_random_walk_);
  } else {
    stream << "not observed (saved as zero)";
  }
  message_ = stream.str();
  plot_.set_sample_count(static_cast<int>(samples.size()));
  plot_.set_sample_rate(sample_rate);
  return true;
}


}  // namespace ssct_imu_intrinsic_calib
