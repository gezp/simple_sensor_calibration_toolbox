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

#include "ssct_cam_imu_calib/handeye_calib/imu_integration.hpp"

#include <sophus/so3.hpp>

namespace ssct_cam_imu_calib
{

void ImuIntegrator::reset(const ImuNavState & state, const ssct_common::ImuData & sample)
{
  state_ = state;
  state_.time = sample.time;
  last_ = sample;
}

bool ImuIntegrator::integrate(const ssct_common::ImuData & sample)
{
  const double dt = sample.time - last_.time;
  if (dt <= 0.0) {
    return false;
  }
  if (dt > 0.5) {
    last_ = sample;
    return false;
  }
  const Eigen::Vector3d omega0 = last_.angular_velocity - state_.gyro_bias;
  const Eigen::Vector3d omega1 = sample.angular_velocity - state_.gyro_bias;
  const Eigen::Matrix3d orientation = state_.orientation *
    Sophus::SO3d::exp(0.5 * (omega0 + omega1) * dt).matrix();
  const Eigen::Vector3d accel0 = last_.linear_acceleration - state_.accel_bias;
  const Eigen::Vector3d accel1 = sample.linear_acceleration - state_.accel_bias;
  const Eigen::Vector3d acceleration =
    0.5 * (state_.orientation * accel0 + orientation * accel1) + state_.gravity;
  const Eigen::Vector3d velocity = state_.linear_velocity + acceleration * dt;
  state_.position += 0.5 * (state_.linear_velocity + velocity) * dt;
  state_.linear_velocity = velocity;
  state_.orientation = orientation;
  state_.time = sample.time;
  last_ = sample;
  return true;
}

}  // namespace ssct_cam_imu_calib
