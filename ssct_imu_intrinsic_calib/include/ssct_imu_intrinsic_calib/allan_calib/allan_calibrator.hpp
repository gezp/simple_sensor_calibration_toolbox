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
#include "ssct_imu_intrinsic_calib/allan_calib/allan_plot.hpp"
#include "ssct_imu_intrinsic_calib/imu_calibrator_interface.hpp"

namespace ssct_imu_intrinsic_calib
{

// Estimate the IMU noise model from a stationary record using the overlapping
// Allan variance. The deterministic affine terms are not estimated here.
class AllanCalibrator : public ImuCalibratorInterface
{
public:
  explicit AllanCalibrator(const YAML::Node & config);
  cv::Mat get_new_debug_image() const override;
  void reset() override;
  bool add_data(const std::vector<ssct_common::ImuData> & samples) override;
  bool is_ready() const override;
  bool optimize() override;
  uint8_t progress() const override {return progress_;}
  const std::string & message() const override {return message_;}
  ssct_common::ImuIntrinsicParam get_result() const override;

private:
  bool estimate_allan(const std::vector<ssct_common::ImuData> & samples);

private:
  // Required stationary duration; collection stops when it is reached.
  double duration_{1800.0};
  // Recompute and redraw the Allan curve every this many seconds.
  double plot_interval_{300.0};
  int num_clusters_{40};
  int min_samples_{1000};
  std::vector<ssct_common::ImuData> samples_;
  AllanPlot plot_;
  uint8_t progress_{0};
  std::string message_;
  Eigen::Vector3d accel_noise_density_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel_random_walk_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gyro_noise_density_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gyro_random_walk_{Eigen::Vector3d::Zero()};
  bool accel_random_walk_valid_{false};
  bool gyro_random_walk_valid_{false};
  double last_update_duration_{-1.0};
};

}  // namespace ssct_imu_intrinsic_calib
