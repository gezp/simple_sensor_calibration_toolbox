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

struct CameraIntrinsicParam
{
  std::string frame_id;
  int height;
  int width;
  std::string type;
  std::vector<double> intrinsics;
  std::vector<double> distortion_coeffs;
};

}  // namespace ssct_common
