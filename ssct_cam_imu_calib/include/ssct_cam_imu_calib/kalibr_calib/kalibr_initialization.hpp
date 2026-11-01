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

#include <array>
#include <vector>

#include "ssct_cam_imu_calib/aruco_detector.hpp"
#include "ssct_cam_imu_calib/kalibr_calib/pose_spline.hpp"
#include "ssct_common/sensor_data/imu_data.hpp"

namespace ssct_cam_imu_calib
{

// Standard gravity magnitude used as the gravity prior (Kalibr default).
constexpr double kStandardGravity = 9.80655;

// Estimates the camera-IMU time offset by matching the board angular speed
// magnitude with the gyro angular speed magnitude. The magnitude is invariant
// to the unknown camera-IMU rotation, so the offset can be estimated first. The
// search covers [-range, range]; zero is returned when the best match falls on
// the search bound (unreliable estimate). The returned value follows
// t_imu = t_camera + time_offset.
double estimate_time_offset(
  const std::vector<BoardObservation> & observations,
  const std::vector<ssct_common::ImuData> & imu, double range, double step);

// Kalibr-style orientation prior: fits the camera-IMU rotation and gyro bias so
// that the gyro measurements match the board angular velocity, rotated into the
// body frame. The translation is not observable from gyro/vision and is left
// to the caller.
bool estimate_orientation_prior(
  const std::vector<BoardObservation> & observations,
  const std::vector<ssct_common::ImuData> & imu, double time_offset,
  const Eigen::Matrix3d & initial_R_body_camera,
  Eigen::Matrix3d & R_body_camera, Eigen::Vector3d & gyro_bias);

// Initializes the control points of the pose spline from the board observations
// (interpolated in time, so no still IMU start is required) and the world
// gravity from the mean specific force, with the direction from the data and
// the magnitude fixed by the caller.
bool build_pose_spline(
  const std::vector<BoardObservation> & observations,
  const std::vector<ssct_common::ImuData> & imu,
  const Eigen::Matrix4d & T_camera_imu,
  double time_offset, double max_time_offset, double knot_spacing,
  double gravity_magnitude,
  PoseSpline & spline, std::vector<std::array<double, 6>> & controls,
  Eigen::Vector3d & gravity_world);

}  // namespace ssct_cam_imu_calib
