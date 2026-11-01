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

#pragma once

#include <yaml-cpp/yaml.h>

#include <Eigen/Geometry>

#include <deque>
#include <vector>

#include "ssct_common/sensor_data/imu_data.hpp"

namespace ssct_cam_imu_calib
{

// A still interval and the mean IMU reading while still. The mean gyro is the
// gyro bias, and gravity is the negated mean specific force in this body frame.
struct StaticImuSegment
{
  double start{0.0};
  double end{0.0};
  Eigen::Vector3d gyro_bias{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gravity{Eigen::Vector3d(0.0, 0.0, -9.80655)};
};

// Collects one still IMU segment at a time. A run is reported only when it has
// ended (motion was observed) and lasted at least min_duration; after that no
// input is accepted until reset().
class StaticImuCollector
{
public:
  StaticImuCollector() = default;
  explicit StaticImuCollector(const YAML::Node & config);

  // Returns true once, when a still run of at least min_duration ends; the
  // segment is available through segment().
  bool collect(const std::vector<ssct_common::ImuData> & imu);
  const StaticImuSegment & segment() const {return segment_;}
  // True once the current still run has lasted min_duration, and stays true
  // after the run ends until reset().
  bool ready() const
  {
    return state_ == State::kDone ||
           (state_ == State::kStill && still_duration() >= min_duration_);
  }
  void reset();

private:
  enum class State {kWaiting, kStill, kDone};

  static constexpr double kGravity = 9.80655;  // m/s^2

  double still_duration() const {return run_end_ - run_start_;}
  bool window_ready(double time) const;
  bool window_is_still() const;
  void update_segment();
  void accumulate(const ssct_common::ImuData & sample);

  double window_{0.5};
  double gyro_std_{0.02};          // rad/s
  double accel_std_{0.2};          // m/s^2
  double gravity_tolerance_{0.5};  // m/s^2, ||accel| - g|
  double min_duration_{0.5};

  StaticImuSegment segment_;
  std::deque<ssct_common::ImuData> window_samples_;
  State state_{State::kWaiting};
  double run_start_{0.0};
  double run_end_{0.0};
  Eigen::Vector3d run_gyro_sum_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d run_accel_sum_{Eigen::Vector3d::Zero()};
  size_t run_count_{0};
};

}  // namespace ssct_cam_imu_calib
