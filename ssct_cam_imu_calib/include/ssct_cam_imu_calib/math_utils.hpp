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

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <vector>

#include "ssct_common/sensor_data/imu_data.hpp"

namespace ssct_cam_imu_calib
{

inline Eigen::Matrix4d interpolate_pose(
  const Eigen::Matrix4d & pose0, const Eigen::Matrix4d & pose1, double alpha)
{
  Eigen::Quaterniond q0(pose0.block<3, 3>(0, 0));
  Eigen::Quaterniond q1(pose1.block<3, 3>(0, 0));
  q0.normalize();
  q1.normalize();
  if (q0.dot(q1) < 0.0) {
    q1.coeffs() *= -1.0;
  }
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  pose.block<3, 3>(0, 0) = q0.slerp(alpha, q1).normalized().toRotationMatrix();
  pose.block<3, 1>(0, 3) =
    (1.0 - alpha) * pose0.block<3, 1>(0, 3) + alpha * pose1.block<3, 1>(0, 3);
  return pose;
}

inline double rotation_angle(const Eigen::Matrix3d & rotation)
{
  return Eigen::AngleAxisd(rotation).angle();
}

inline double degrees_to_radians(double degrees)
{
  return degrees * 0.017453292519943295;
}

inline double radians_to_degrees(double radians)
{
  return radians * 57.29577951308232;
}

// Maximum rotation angle of `poses` relative to the first pose.
inline double rotation_span(const std::vector<Eigen::Matrix4d> & poses)
{
  if (poses.empty()) {
    return 0.0;
  }
  const Eigen::Matrix3d reference = poses.front().block<3, 3>(0, 0);
  double span = 0.0;
  for (const auto & pose : poses) {
    span = std::max(
      span, rotation_angle(reference.transpose() * pose.block<3, 3>(0, 0)));
  }
  return span;
}

inline bool is_finite(const Eigen::Matrix4d & pose)
{
  return pose.allFinite();
}

inline bool is_finite(const Eigen::Vector3d & vector)
{
  return vector.allFinite();
}

// Linearly interpolates an IMU sample at `time`. Returns false when `time` is
// outside the sample range.
inline bool interpolate_imu_sample(
  const std::vector<ssct_common::ImuData> & imu, double time,
  ssct_common::ImuData & sample)
{
  constexpr double epsilon = 1e-9;
  if (imu.empty() || time < imu.front().time - epsilon ||
    time > imu.back().time + epsilon)
  {
    return false;
  }
  if (time <= imu.front().time) {
    sample = imu.front();
    sample.time = time;
    return true;
  }
  if (time >= imu.back().time) {
    sample = imu.back();
    sample.time = time;
    return true;
  }
  auto it = std::lower_bound(
    imu.begin(), imu.end(), time,
    [](const ssct_common::ImuData & item, double value) {return item.time < value;});
  if (it == imu.begin()) {
    sample = *it;
    sample.time = time;
    return true;
  }
  const auto & right = *it;
  const auto & left = *(it - 1);
  const double dt = right.time - left.time;
  const double alpha = dt > epsilon ? (time - left.time) / dt : 0.0;
  sample.time = time;
  sample.angular_velocity =
    (1.0 - alpha) * left.angular_velocity + alpha * right.angular_velocity;
  sample.linear_acceleration =
    (1.0 - alpha) * left.linear_acceleration + alpha * right.linear_acceleration;
  return true;
}

}  // namespace ssct_cam_imu_calib
