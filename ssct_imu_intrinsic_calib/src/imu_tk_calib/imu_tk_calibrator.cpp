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

#include "ssct_imu_intrinsic_calib/imu_tk_calib/imu_tk_calibrator.hpp"

#include <Eigen/Geometry>

#include <ceres/ceres.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <utility>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "ssct_common/calib_param/imu_intrinsic_param.hpp"
#include "ssct_imu_intrinsic_calib/imu_tk_calib/imu_factor.hpp"
#include "ssct_imu_intrinsic_calib/imu_tk_calib/imu_tk_plot.hpp"
#include "ssct_imu_intrinsic_calib/utils.hpp"

namespace ssct_imu_intrinsic_calib
{
namespace
{

Eigen::Matrix3d accel_matrix(const Eigen::VectorXd & p)
{
  Eigen::Matrix3d misalignment;
  misalignment << 1.0, 0.0, 0.0,
    p[0], 1.0, 0.0,
    -p[1], p[2], 1.0;
  const Eigen::Matrix3d scale = p.segment<3>(3).asDiagonal();
  return misalignment * scale;
}

Eigen::Matrix3d gyro_matrix(const Eigen::VectorXd & p)
{
  Eigen::Matrix3d misalignment;
  misalignment << 1.0, -p[0], p[1],
    p[3], 1.0, -p[2],
    -p[4], p[5], 1.0;
  const Eigen::Matrix3d scale = p.segment<3>(6).asDiagonal();
  return misalignment * scale;
}

std::vector<Eigen::Vector3d> extract_accel_samples(
  const std::vector<ssct_common::ImuData> & samples,
  const std::vector<StaticInterval> & intervals, int num_samples)
{
  std::vector<Eigen::Vector3d> extracted;
  extracted.reserve(intervals.size() * static_cast<size_t>(num_samples));
  for (const auto & interval : intervals) {
    for (int i = interval.start; i < interval.start + num_samples; i++) {
      extracted.push_back(samples[i].linear_acceleration);
    }
  }
  return extracted;
}

}  // namespace

ImuTkCalibrator::ImuTkCalibrator(const YAML::Node & config)
{
  if (!config) {
    return;
  }
  if (config["gravity_magnitude"]) {
    gravity_magnitude_ = config["gravity_magnitude"].as<double>();
  }
  double static_window = 0.5;
  int interval_samples = 100;
  int min_intervals = 9;
  if (config["static_window"]) {
    static_window = config["static_window"].as<double>();
  }
  if (config["interval_samples"]) {
    interval_samples = config["interval_samples"].as<int>();
  }
  if (config["min_intervals"]) {
    min_intervals = config["min_intervals"].as<int>();
  }
  collector_ = StaticPoseCollector(static_window, interval_samples, min_intervals);
}

cv::Mat ImuTkCalibrator::get_new_debug_image() const
{
  const cv::Mat bgr = plot_.draw();
  cv::Mat rgb;
  cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
  return rgb;
}

void ImuTkCalibrator::reset()
{
  samples_.clear();
  progress_ = 0;
  message_ = "start calibration: hold each pose still until the status says to "
    "move, then rotate to a new pose.";
  accel_matrix_ = Eigen::Matrix3d::Identity();
  accel_bias_.setZero();
  gyro_matrix_ = Eigen::Matrix3d::Identity();
  gyro_bias_.setZero();
  plot_.clear_data();
  collector_.reset();
  last_attempt_duration_ = -1.0;
}

bool ImuTkCalibrator::add_data(
  const std::vector<ssct_common::ImuData> & samples)
{
  constexpr double kDetectionInterval = 1.0;
  samples_.insert(samples_.end(), samples.begin(), samples.end());
  if (samples_.empty()) {
    message_ = "waiting for IMU data.";
    progress_ = 1;
    return false;
  }
  const double duration = samples_.back().time - samples_.front().time;
  const bool detect = last_attempt_duration_ < 0.0 ||
    duration - last_attempt_duration_ >= kDetectionInterval;
  if (detect) {
    last_attempt_duration_ = duration;
    std::vector<Eigen::Vector3d> accel;
    accel.reserve(samples_.size());
    for (const auto & sample : samples_) {
      accel.push_back(sample.linear_acceleration);
    }
    plot_.set_samples(accel);
    collector_.collect(samples_);
    std::vector<std::pair<int, int>> intervals;
    for (const auto & interval : collector_.display_intervals()) {
      intervals.emplace_back(interval.start, interval.end);
    }
    plot_.set_intervals(intervals);
    plot_.set_threshold_multiplier(0);
  }
  progress_ = 10;
  std::ostringstream stream;
  stream << collector_.progress() << ", " << collector_.hint()
         << ", " << duration << " s, " << samples_.size() << " samples";
  message_ = stream.str();
  return detect;
}

bool ImuTkCalibrator::is_ready() const
{
  for (const auto & candidate : collector_.candidates()) {
    if (static_cast<int>(candidate.intervals.size()) >= collector_.min_intervals()) {
      return true;
    }
  }
  return false;
}

bool ImuTkCalibrator::optimize()
{
  progress_ = 100;
  return estimate();
}

ssct_common::ImuIntrinsicParam ImuTkCalibrator::get_result() const
{
  ssct_common::ImuIntrinsicParam result;
  result.frame_id = frame_id_;
  result.accel_matrix = accel_matrix_;
  result.accel_bias = accel_bias_;
  result.accel_noise_density.setZero();
  result.accel_random_walk.setZero();
  result.gyro_matrix = gyro_matrix_;
  result.gyro_bias = gyro_bias_;
  result.gyro_noise_density.setZero();
  result.gyro_random_walk.setZero();
  return result;
}

bool ImuTkCalibrator::estimate()
{
  message_.clear();
  if (samples_.size() < 20 || !(gravity_magnitude_ > 0.0)) {
    message_ = "not enough samples or invalid TK configuration";
    plot_.set_intervals({});
    plot_.set_threshold_multiplier(0);
    return false;
  }
  if (!collector_.valid()) {
    message_ = collector_.message();
    plot_.set_intervals({});
    plot_.set_threshold_multiplier(0);
    return false;
  }

  Eigen::VectorXd best_accel_params;
  std::vector<StaticInterval> best_intervals;
  double best_cost = std::numeric_limits<double>::max();
  int best_multiplier = 0;
  for (const auto & candidate : collector_.candidates()) {
    if (static_cast<int>(candidate.intervals.size()) < collector_.min_intervals()) {
      continue;
    }
    const auto accel_samples = extract_accel_samples(
      samples_, candidate.intervals, collector_.interval_samples());
    double cost = 0.0;
    const Eigen::VectorXd params = solve_accel(accel_samples, &cost);
    if (cost < best_cost) {
      best_cost = cost;
      best_multiplier = candidate.threshold_multiplier;
      best_accel_params = params;
      best_intervals = candidate.intervals;
    }
  }
  if (best_multiplier == 0) {
    std::ostringstream stream;
    stream << "not enough static intervals, " << samples_.size() << " samples";
    message_ = stream.str();
    std::vector<std::pair<int, int>> intervals;
    for (const auto & interval : collector_.display_intervals()) {
      intervals.emplace_back(interval.start, interval.end);
    }
    plot_.set_intervals(intervals);
    plot_.set_threshold_multiplier(0);
    return false;
  }

  const Eigen::Matrix3d accel_mat = accel_matrix(best_accel_params);
  const Eigen::Vector3d raw_accel_bias = best_accel_params.segment<3>(6);
  std::vector<std::pair<int, int>> intervals;
  for (const auto & interval : best_intervals) {
    intervals.emplace_back(interval.start, interval.end);
  }

  const auto transitions = build_transitions(
    samples_, best_intervals, accel_mat, raw_accel_bias);
  if (transitions.empty()) {
    message_ = "no valid rotation between static intervals";
    plot_.set_intervals(intervals);
    plot_.set_threshold_multiplier(best_multiplier);
    return false;
  }
  Eigen::Vector3d raw_gyro_bias = Eigen::Vector3d::Zero();
  for (const auto & interval : best_intervals) {
    raw_gyro_bias += interval_gyro_mean(samples_, interval.start, interval.end);
  }
  raw_gyro_bias /= static_cast<double>(best_intervals.size());
  double gyro_cost = 0.0;
  const Eigen::VectorXd gyro_params =
    solve_gyro(samples_, transitions, raw_gyro_bias, &gyro_cost);
  const Eigen::Matrix3d gyro_mat = gyro_matrix(gyro_params);

  accel_matrix_ = accel_mat;
  accel_bias_ = raw_accel_bias;
  gyro_matrix_ = gyro_mat;
  gyro_bias_ = raw_gyro_bias;
  ssct_common::ImuIntrinsicParam result;
  result.frame_id = frame_id_;
  result.accel_matrix = accel_matrix_;
  result.accel_bias = accel_bias_;
  result.gyro_matrix = gyro_matrix_;
  result.gyro_bias = gyro_bias_;
  std::ostringstream stream;
  stream << "samples: " << samples_.size() << "\n";
  stream << "duration: " << samples_.back().time - samples_.front().time << " s\n";
  stream << "static_intervals: " << best_intervals.size()
         << " (threshold x" << best_multiplier << ")\n";
  stream << "accel_rms: " << std::sqrt(best_cost) << "\n";
  stream << "gyro_rms: " << std::sqrt(gyro_cost) << "\n";
  message_ = stream.str() + ssct_common::to_string(result);
  plot_.set_intervals(intervals);
  plot_.set_threshold_multiplier(best_multiplier);
  return true;
}

Eigen::VectorXd ImuTkCalibrator::solve_accel(
  const std::vector<Eigen::Vector3d> & samples,
  double * mean_squared_error) const
{
  // Initialize from the data so raw sensor counts work as well as SI units.
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const auto & sample : samples) {
    mean += sample;
  }
  mean /= static_cast<double>(samples.size());
  double mean_norm = 0.0;
  for (const auto & sample : samples) {
    mean_norm += (sample - mean).norm();
  }
  mean_norm /= static_cast<double>(samples.size());
  const double scale = mean_norm > 0.0 ? gravity_magnitude_ / mean_norm : 1.0;
  double params[9] = {
    0.0, 0.0, 0.0, scale, scale, scale, mean.x(), mean.y(), mean.z()};
  ceres::Problem problem;
  for (const auto & sample : samples) {
    problem.AddResidualBlock(
      new AccelCost(sample[0], sample[1], sample[2], gravity_magnitude_),
      nullptr, params);
  }
  for (int i = 0; i < 3; i++) {
    problem.SetParameterLowerBound(params, i, -0.5);
    problem.SetParameterUpperBound(params, i, 0.5);
  }
  for (int i = 3; i < 6; i++) {
    problem.SetParameterLowerBound(params, i, 1e-9);
    problem.SetParameterUpperBound(params, i, 1e3);
  }
  ceres::Solver::Summary summary;
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.max_num_iterations = 100;
  options.function_tolerance = 1e-12;
  options.gradient_tolerance = 1e-12;
  options.parameter_tolerance = 1e-12;
  options.logging_type = ceres::SILENT;
  options.minimizer_progress_to_stdout = false;
  ceres::Solve(options, &problem, &summary);
  if (summary.termination_type == ceres::FAILURE) {
    *mean_squared_error = std::numeric_limits<double>::infinity();
  } else {
    *mean_squared_error =
      2.0 * summary.final_cost / static_cast<double>(samples.size());
  }
  return Eigen::Map<Eigen::VectorXd>(params, 9);
}

std::vector<ImuTkCalibrator::Transition> ImuTkCalibrator::build_transitions(
  const std::vector<ssct_common::ImuData> & samples,
  const std::vector<StaticInterval> & intervals,
  const Eigen::Matrix3d & accel_matrix, const Eigen::Vector3d & accel_bias) const
{
  std::vector<Transition> transitions;
  if (intervals.size() < 2) {
    return transitions;
  }
  std::vector<Eigen::Vector3d> gravities;
  gravities.reserve(intervals.size());
  for (const auto & interval : intervals) {
    Eigen::Vector3d gravity_vector =
      accel_matrix *
      (interval_accel_mean(samples, interval.start, interval.end) - accel_bias);
    const double norm = gravity_vector.norm();
    if (norm <= 0.0) {
      return {};
    }
    // Keep the direction; the magnitude is fixed to the calibrated gravity.
    gravities.push_back(gravity_vector / norm);
  }
  int search = 0;
  for (size_t i = 0; i + 1 < intervals.size(); i++) {
    const double t_0 = samples[intervals[i].end].time;
    const double t_1 = samples[intervals[i + 1].start].time;
    while (search < static_cast<int>(samples.size()) && samples[search].time < t_0) {
      search++;
    }
    int start = search;
    while (search < static_cast<int>(samples.size()) && samples[search].time < t_1) {
      search++;
    }
    const int end = search;  // exclusive
    if (end - start < 2) {
      continue;
    }
    Transition transition;
    transition.start = start;
    transition.end = end;
    transition.gravity_0 = gravities[i];
    transition.gravity_1 = gravities[i + 1];
    transitions.push_back(transition);
  }
  return transitions;
}

Eigen::VectorXd ImuTkCalibrator::solve_gyro(
  const std::vector<ssct_common::ImuData> & samples,
  const std::vector<Transition> & transitions, const Eigen::Vector3d & bias,
  double * mean_squared_error) const
{
  // Estimate a uniform scale from the angle between adjacent static poses.
  double angle_sum = 0.0;
  double rate_sum = 0.0;
  for (const auto & transition : transitions) {
    Eigen::Vector3d integrated = Eigen::Vector3d::Zero();
    for (int i = transition.start; i + 1 < transition.end; i++) {
      const double dt = samples[i + 1].time - samples[i].time;
      integrated += (samples[i].angular_velocity - bias) * dt;
    }
    const double cosine = std::clamp(
      transition.gravity_0.dot(transition.gravity_1), -1.0, 1.0);
    const double angle = std::acos(cosine);
    const double rate = integrated.norm();
    if (angle > 0.0 && rate > 0.0) {
      angle_sum += angle;
      rate_sum += rate;
    }
  }
  const double gyro_scale = rate_sum > 0.0 ? angle_sum / rate_sum : 1.0;
  double params[9] = {
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, gyro_scale, gyro_scale, gyro_scale};
  ceres::Problem problem;
  for (const auto & transition : transitions) {
    problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<GyroCost, 3, 9>(
        new GyroCost(
          samples, transition.start, transition.end, transition.gravity_0,
          transition.gravity_1, bias)),
      nullptr, params);
  }
  for (int i = 0; i < 6; i++) {
    problem.SetParameterLowerBound(params, i, -0.5);
    problem.SetParameterUpperBound(params, i, 0.5);
  }
  for (int i = 6; i < 9; i++) {
    problem.SetParameterLowerBound(params, i, 1e-9);
    problem.SetParameterUpperBound(params, i, 1e3);
  }
  ceres::Solver::Summary summary;
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.max_num_iterations = 100;
  options.function_tolerance = 1e-12;
  options.gradient_tolerance = 1e-12;
  options.parameter_tolerance = 1e-12;
  options.logging_type = ceres::SILENT;
  options.minimizer_progress_to_stdout = false;
  ceres::Solve(options, &problem, &summary);
  if (summary.termination_type == ceres::FAILURE) {
    *mean_squared_error = std::numeric_limits<double>::infinity();
  } else {
    *mean_squared_error = 2.0 * summary.final_cost /
      static_cast<double>(3 * transitions.size());
  }
  return Eigen::Map<Eigen::VectorXd>(params, 9);
}


}  // namespace ssct_imu_intrinsic_calib
