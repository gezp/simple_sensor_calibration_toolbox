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

#include <vector>

#include "ssct_common/sensor_data/imu_data.hpp"

namespace ssct_imu_intrinsic_calib
{

Eigen::Vector3d interval_accel_mean(
  const std::vector<ssct_common::ImuData> & samples, int start, int end);

Eigen::Vector3d interval_gyro_mean(
  const std::vector<ssct_common::ImuData> & samples, int start, int end);

}  // namespace ssct_imu_intrinsic_calib
