// Copyright 2025 Gezp (https://github.com/gezp).
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

#include <string>
#include <vector>

namespace ssct_common
{

struct ImuIntrinsicParam
{
  std::string frame_id;
  // for accelerometer
  Eigen::Matrix3d accel_matrix;
  Eigen::Vector3d accel_offset;
  Eigen::Vector3d accel_noise_density;
  Eigen::Vector3d accel_random_walk;
  // for gyroscope
  Eigen::Matrix3d gyro_matrix;
  Eigen::Vector3d gyro_offset;
  Eigen::Vector3d gyro_noise_density;
  Eigen::Vector3d gyro_random_walk;
};

}  // namespace ssct_common
