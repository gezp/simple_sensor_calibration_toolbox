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

#include <cstddef>
#include <string>
#include <vector>

namespace ssct_common
{

struct CameraIntrinsicParam
{
  std::string frame_id;
  int height;
  int width;
  std::string type;
  std::vector<double> intrinsics;
  std::vector<double> distortion_coeffs;
};

std::string to_string(const CameraIntrinsicParam & param);

// Canonical models:
//   pinhole          [fx, fy, cx, cy]                 no distortion
//   pinhole_radtan   [fx, fy, cx, cy]                 [k1, k2, p1, p2, k3]
//   ftheta           [fx, fy, cx, cy]                 no distortion
//   ftheta_kb        [fx, fy, cx, cy]                 [k1, k2, k3, k4]
//   mei              [xi, fx, fy, cx, cy]             no distortion
//   mei_radtan       [xi, fx, fy, cx, cy]             [k1, k2, p1, p2]
inline bool normalize_camera_model(std::string & type)
{
  if (type == "fisheye") {
    type = "ftheta_kb";
    return true;
  }
  if (type == "omnidir") {
    type = "mei_radtan";
    return true;
  }
  return type == "pinhole" || type == "pinhole_radtan" ||
         type == "ftheta" || type == "ftheta_kb" ||
         type == "mei" || type == "mei_radtan";
}

inline bool is_pinhole_camera_model(const std::string & type)
{
  return type == "pinhole" || type == "pinhole_radtan";
}

inline bool is_ftheta_camera_model(const std::string & type)
{
  return type == "ftheta" || type == "ftheta_kb";
}

inline bool is_mei_camera_model(const std::string & type)
{
  return type == "mei" || type == "mei_radtan";
}

inline bool is_valid_camera_intrinsic_param(const CameraIntrinsicParam & param)
{
  std::string type = param.type;
  if (param.frame_id.empty() || param.width <= 0 || param.height <= 0 ||
    !normalize_camera_model(type))
  {
    return false;
  }
  size_t intrinsic_count = 4;
  size_t distortion_count = 0;
  if (type == "pinhole_radtan") {
    distortion_count = 5;
  } else if (type == "ftheta_kb") {
    distortion_count = 4;
  } else if (type == "mei" || type == "mei_radtan") {
    intrinsic_count = 5;
    distortion_count = type == "mei_radtan" ? 4 : 0;
  }
  return param.intrinsics.size() == intrinsic_count &&
         param.distortion_coeffs.size() == distortion_count;
}

}  // namespace ssct_common
