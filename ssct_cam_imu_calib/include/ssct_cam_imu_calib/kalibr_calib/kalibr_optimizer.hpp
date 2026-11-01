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

#include <array>
#include <limits>
#include <string>
#include <vector>

#include "ssct_cam_imu_calib/aruco_detector.hpp"
#include "ssct_cam_imu_calib/camera_model.hpp"
#include "ssct_cam_imu_calib/kalibr_calib/pose_spline.hpp"
#include "ssct_common/calib_param/extrinsic_param.hpp"
#include "ssct_common/calib_param/imu_intrinsic_param.hpp"
#include "ssct_common/sensor_data/imu_data.hpp"

namespace ssct_cam_imu_calib
{

// Camera-IMU state: extrinsic, biases, time offset, gravity and the pose-spline
// control points. Used both as the initial guess and the optimized result.
struct KalibrEstimatedState
{
  Eigen::Matrix4d T_camera_imu{Eigen::Matrix4d::Identity()};
  Eigen::Vector3d gyro_bias{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel_bias{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gravity{Eigen::Vector3d::Zero()};
  // t_imu = t_camera + time_offset (calibration-file convention).
  double time_offset{0.0};
  PoseSpline spline;
  std::vector<std::array<double, 6>> controls;
};

struct Evaluation
{
  double reprojection_rms{std::numeric_limits<double>::infinity()};
  double reprojection_median{std::numeric_limits<double>::infinity()};
  double reprojection_p95{std::numeric_limits<double>::infinity()};
  int bad_frames{0};
  double gyro_rms{0.0};
  double accel_rms{0.0};
};

class KalibrOptimizer
{
public:
  KalibrOptimizer() = default;
  explicit KalibrOptimizer(const YAML::Node & config);

  void set_camera_model(const CameraModel & camera_model);
  void set_imu_intrinsic_param(const ssct_common::ImuIntrinsicParam & imu);
  void add_data(
    const std::vector<BoardObservation> & observations,
    const std::vector<ssct_common::ImuData> & imu_samples);

  bool optimize();
  const ssct_common::ExtrinsicParam & get_result() const {return result_;}
  const std::string & message() const {return message_;}

private:
  bool build_initial_guess(KalibrEstimatedState & guess);

  bool solve_problem(
    const KalibrEstimatedState & initial_state,
    KalibrEstimatedState & final_state);

  Evaluation evaluate(const KalibrEstimatedState & state) const;

private:
  const CameraModel * camera_model_{nullptr};
  std::vector<BoardObservation> observations_;
  std::vector<ssct_common::ImuData> imu_samples_;
  ssct_common::ExtrinsicParam result_;
  int max_iterations_{60};
  double spline_knot_spacing_{0.05};
  double reprojection_sigma_px_{1.0};
  double max_reprojection_rms_{2.0};
  double gyro_noise_density_{0.0};
  double accel_noise_density_{0.0};
  double gyro_bias_sigma_{0.05};
  double accel_bias_sigma_{0.5};
  double time_offset_sigma_{0.05};
  double max_time_offset_{0.1};
  double gravity_sigma_{0.5};
  bool estimate_time_offset_{true};
  bool estimate_gravity_{true};
  std::string solver_message_;
  std::string message_;
};

}  // namespace ssct_cam_imu_calib
