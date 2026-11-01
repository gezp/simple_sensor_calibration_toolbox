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

#include <string>
#include <vector>

#include "ssct_common/calib_param/extrinsic_param.hpp"

namespace ssct_cam_imu_calib
{

// Relative motion of one A->B still pair in each sensor frame. The IMU part is
// integrated once, with the gyro bias and gravity estimated inside segment A.
struct HandEyePair
{
  Eigen::Matrix4d T_cam_a_b{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d T_imu_a_b{Eigen::Matrix4d::Identity()};
  double position_drift{0.0};  // |position| of T_imu_a_b
};

class HandEyeOptimizer
{
public:
  HandEyeOptimizer() = default;
  explicit HandEyeOptimizer(const YAML::Node & config);

  void add_data(const std::vector<HandEyePair> & pairs);
  bool optimize();
  const std::string & message() const {return message_;}
  ssct_common::ExtrinsicParam get_result() const {return result_;}

private:
  std::vector<HandEyePair> pairs_;
  ssct_common::ExtrinsicParam result_;
  double max_score_{0.1};
  int max_pairs_{200};
  std::string message_;
};

}  // namespace ssct_cam_imu_calib
