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

#include <Eigen/Geometry>

#include "ssct_common/sensor_data/imu_data.hpp"

namespace ssct_cam_imu_calib
{

struct ImuNavState
{
  double time{0.0};
  Eigen::Matrix3d orientation{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d linear_velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gyro_bias{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel_bias{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gravity{Eigen::Vector3d(0.0, 0.0, -9.80665)};
};

// Midpoint IMU integration, adapted from slt_imu_odometry::ImuIntegration.
class ImuIntegrator
{
public:
  void reset(const ImuNavState & state, const ssct_common::ImuData & sample);
  bool integrate(const ssct_common::ImuData & sample);
  const ImuNavState & state() const {return state_;}

private:
  ImuNavState state_;
  ssct_common::ImuData last_;
};

}  // namespace ssct_cam_imu_calib
