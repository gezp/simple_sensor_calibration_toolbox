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

#include "ssct_common/calib_param/imu_intrinsic_param.hpp"

#include <iomanip>
#include <sstream>

namespace ssct_common
{
namespace
{

std::string format_values(const double * values, size_t size)
{
  std::ostringstream stream;
  stream << std::setprecision(9) << "[";
  for (size_t i = 0; i < size; i++) {
    if (i > 0) {
      stream << ", ";
    }
    stream << values[i];
  }
  stream << "]";
  return stream.str();
}

}  // namespace

std::string to_string(const ImuIntrinsicParam & param)
{
  std::ostringstream stream;
  stream << "frame_id: " << param.frame_id << "\n";
  stream << "accel_matrix: " << format_values(param.accel_matrix.data(), 9) << "\n";
  stream << "accel_bias: " << format_values(param.accel_bias.data(), 3) << "\n";
  stream << "accel_noise_density: "
         << format_values(param.accel_noise_density.data(), 3) << "\n";
  stream << "accel_random_walk: "
         << format_values(param.accel_random_walk.data(), 3) << "\n";
  stream << "gyro_matrix: " << format_values(param.gyro_matrix.data(), 9) << "\n";
  stream << "gyro_bias: " << format_values(param.gyro_bias.data(), 3) << "\n";
  stream << "gyro_noise_density: "
         << format_values(param.gyro_noise_density.data(), 3) << "\n";
  stream << "gyro_random_walk: " << format_values(param.gyro_random_walk.data(), 3);
  return stream.str();
}

}  // namespace ssct_common
