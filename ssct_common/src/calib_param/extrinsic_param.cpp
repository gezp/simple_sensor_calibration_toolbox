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

#include "ssct_common/calib_param/extrinsic_param.hpp"

#include <Eigen/Geometry>

#include <iomanip>
#include <sstream>

namespace ssct_common
{

std::string to_string(const ExtrinsicParam & param)
{
  const Eigen::Matrix3d rotation = param.transform.block<3, 3>(0, 0);
  const Eigen::Quaterniond quaternion(rotation);
  std::ostringstream stream;
  stream << std::setprecision(9);
  stream << "frame_id: " << param.frame_id << "\n";
  stream << "child_frame_id: " << param.child_frame_id << "\n";
  stream << "translation: ["
         << param.transform(0, 3) << ", " << param.transform(1, 3) << ", "
         << param.transform(2, 3) << "]\n";
  stream << "rotation_xyzw: ["
         << quaternion.x() << ", " << quaternion.y() << ", "
         << quaternion.z() << ", " << quaternion.w() << "]";
  return stream.str();
}

}  // namespace ssct_common
