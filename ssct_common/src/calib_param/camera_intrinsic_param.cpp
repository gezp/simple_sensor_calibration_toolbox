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

#include "ssct_common/calib_param/camera_intrinsic_param.hpp"

#include <iomanip>
#include <sstream>
#include <vector>

namespace ssct_common
{
namespace
{

std::string format_values(const std::vector<double> & values)
{
  std::ostringstream stream;
  stream << std::setprecision(9) << "[";
  for (size_t i = 0; i < values.size(); i++) {
    if (i > 0) {
      stream << ", ";
    }
    stream << values[i];
  }
  stream << "]";
  return stream.str();
}

}  // namespace

std::string to_string(const CameraIntrinsicParam & param)
{
  std::ostringstream stream;
  stream << "frame_id: " << param.frame_id << "\n";
  stream << "height: " << param.height << "\n";
  stream << "width: " << param.width << "\n";
  stream << "type: " << param.type << "\n";
  stream << "intrinsics: " << format_values(param.intrinsics) << "\n";
  stream << "distortion_coeffs: " << format_values(param.distortion_coeffs);
  return stream.str();
}

}  // namespace ssct_common
