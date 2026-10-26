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

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

#include <string>
#include <vector>

#include "ssct_common/sensor_data/imu_data.hpp"
#include "ssct_imu_intrinsic_calib/imu_calibrator_interface.hpp"
#include "ssct_imu_intrinsic_calib/imu_tk_calib/imu_tk_plot.hpp"
#include "ssct_imu_intrinsic_calib/imu_tk_calib/static_pose_collector.hpp"

namespace ssct_imu_intrinsic_calib
{

// Multi-position intrinsic calibration following the IMU-TK method:
// accelerometer scale/misalignment/bias from static gravity observations, then
// gyroscope scale/misalignment/bias from integrated rotations between the
// static poses. Output uses the ssct affine model:
// x_calibrated = M * (x - b).
class ImuTkCalibrator : public ImuCalibratorInterface
{
public:
  explicit ImuTkCalibrator(const YAML::Node & config);
  void set_frame_id(const std::string & frame_id) {frame_id_ = frame_id;}
  cv::Mat get_new_debug_image() const override;
  void reset() override;
  bool add_data(const std::vector<ssct_common::ImuData> & samples) override;
  bool is_ready() const override;
  bool optimize() override;
  uint8_t progress() const override {return progress_;}
  const std::string & message() const override {return message_;}
  ssct_common::ImuIntrinsicParam get_result() const override;

private:
  struct Transition
  {
    int start;
    int end;  // exclusive
    Eigen::Vector3d gravity_0;
    Eigen::Vector3d gravity_1;
  };

  bool estimate();
  Eigen::VectorXd solve_accel(
    const std::vector<Eigen::Vector3d> & samples,
    double * mean_squared_error) const;
  std::vector<Transition> build_transitions(
    const std::vector<ssct_common::ImuData> & samples,
    const std::vector<StaticInterval> & intervals,
    const Eigen::Matrix3d & accel_matrix, const Eigen::Vector3d & accel_bias) const;
  Eigen::VectorXd solve_gyro(
    const std::vector<ssct_common::ImuData> & samples,
    const std::vector<Transition> & transitions, const Eigen::Vector3d & bias,
    double * mean_squared_error) const;

private:
  std::string frame_id_;
  // Local gravity magnitude, used as the accelerometer reference.
  double gravity_magnitude_{9.80665};
  // Detects static poses and reports the keep-still / move prompts.
  StaticPoseCollector collector_;
  ImuTkPlot plot_;
  std::vector<ssct_common::ImuData> samples_;
  uint8_t progress_{0};
  std::string message_;
  Eigen::Matrix3d accel_matrix_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d accel_bias_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d gyro_matrix_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d gyro_bias_{Eigen::Vector3d::Zero()};
  double last_attempt_duration_{-1.0};
};

}  // namespace ssct_imu_intrinsic_calib
