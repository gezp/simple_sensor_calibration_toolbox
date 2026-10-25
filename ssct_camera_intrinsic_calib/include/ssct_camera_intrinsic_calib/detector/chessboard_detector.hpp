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

#include <vector>

#include "opencv2/opencv.hpp"

namespace ssct_camera_intrinsic_calib
{

// detect chessboard corners and provide the matching 3D pattern points
class ChessboardDetector
{
public:
  explicit ChessboardDetector(const YAML::Node & config);
  bool detect(const cv::Mat & image, std::vector<cv::Point2f> & corners) const;
  const cv::Size & get_corner_size() const;
  const std::vector<cv::Point3f> & get_pattern_points() const;

private:
  cv::Size corner_size_;
  std::vector<cv::Point3f> pattern_points_;
};

}  // namespace ssct_camera_intrinsic_calib
