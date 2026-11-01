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

#include "ssct_cam_imu_calib/handeye_calib/static_imu_collector.hpp"

#include <cmath>

namespace ssct_cam_imu_calib
{

StaticImuCollector::StaticImuCollector(const YAML::Node & config)
{
  if (config["window"]) {
    window_ = config["window"].as<double>();
  }
  if (config["gyro_std"]) {
    gyro_std_ = config["gyro_std"].as<double>();
  }
  if (config["accel_std"]) {
    accel_std_ = config["accel_std"].as<double>();
  }
  if (config["gravity_tolerance"]) {
    gravity_tolerance_ = config["gravity_tolerance"].as<double>();
  }
  if (config["min_duration"]) {
    min_duration_ = config["min_duration"].as<double>();
  }
}

bool StaticImuCollector::collect(const std::vector<ssct_common::ImuData> & imu)
{
  if (state_ == State::kDone) {
    return false;
  }
  for (const auto & sample : imu) {
    window_samples_.push_back(sample);
    while (window_samples_.size() > 2 &&
      window_samples_[1].time <= sample.time - window_)
    {
      window_samples_.pop_front();
    }
    if (!window_ready(sample.time)) {
      continue;
    }
    if (!window_is_still()) {
      if (state_ == State::kStill) {
        // Motion ends the run; short runs are dropped and collection continues.
        state_ = State::kWaiting;
        if (still_duration() >= min_duration_) {
          update_segment();
          state_ = State::kDone;
          return true;
        }
      }
      continue;
    }
    if (state_ == State::kWaiting) {
      state_ = State::kStill;
      run_start_ = window_samples_.front().time;
      run_end_ = sample.time;
      run_gyro_sum_.setZero();
      run_accel_sum_.setZero();
      run_count_ = 0;
      for (const auto & item : window_samples_) {
        accumulate(item);
      }
    } else {
      accumulate(sample);
      run_end_ = sample.time;
    }
  }
  return false;
}

void StaticImuCollector::reset()
{
  segment_ = StaticImuSegment();
  window_samples_.clear();
  state_ = State::kWaiting;
  run_start_ = 0.0;
  run_end_ = 0.0;
  run_gyro_sum_.setZero();
  run_accel_sum_.setZero();
  run_count_ = 0;
}

bool StaticImuCollector::window_ready(double time) const
{
  return window_samples_.size() >= 2 &&
         window_samples_.front().time <= time - window_;
}

bool StaticImuCollector::window_is_still() const
{
  Eigen::Vector3d gyro_mean = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel_mean = Eigen::Vector3d::Zero();
  for (const auto & sample : window_samples_) {
    gyro_mean += sample.angular_velocity;
    accel_mean += sample.linear_acceleration;
  }
  const double count = static_cast<double>(window_samples_.size());
  gyro_mean /= count;
  accel_mean /= count;

  double gyro_variance = 0.0;
  double accel_variance = 0.0;
  for (const auto & sample : window_samples_) {
    gyro_variance += (sample.angular_velocity - gyro_mean).squaredNorm();
    accel_variance += (sample.linear_acceleration - accel_mean).squaredNorm();
  }
  const double gyro_std = std::sqrt(gyro_variance / count);
  const double accel_std = std::sqrt(accel_variance / count);
  return gyro_std <= gyro_std_ && accel_std <= accel_std_ &&
         std::abs(accel_mean.norm() - kGravity) <= gravity_tolerance_;
}

void StaticImuCollector::update_segment()
{
  segment_.start = run_start_;
  segment_.end = run_end_;
  segment_.gyro_bias = run_gyro_sum_ / static_cast<double>(run_count_);
  segment_.gravity = -run_accel_sum_ / static_cast<double>(run_count_);
}

void StaticImuCollector::accumulate(const ssct_common::ImuData & sample)
{
  run_gyro_sum_ += sample.angular_velocity;
  run_accel_sum_ += sample.linear_acceleration;
  run_count_++;
}

}  // namespace ssct_cam_imu_calib
