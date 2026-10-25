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

#include <string>
#include <vector>

#include "opencv2/opencv.hpp"

namespace ssct_camera_intrinsic_calib
{

class CalibratorInterface
{
public:
  virtual ~CalibratorInterface() = default;
  virtual bool process_image(const cv::Mat & image) = 0;
  virtual bool ready_to_optimize() = 0;
  virtual bool optimize() = 0;
  virtual void clear() = 0;
  virtual std::string get_type() const = 0;
  virtual std::vector<double> get_intrinsics() const = 0;
  virtual std::vector<double> get_distortion_coeffs() const = 0;
  virtual cv::Size get_image_size() const = 0;
  virtual const std::string & get_status_message() const = 0;
  virtual cv::Mat get_debug_image() const = 0;
};

}  // namespace ssct_camera_intrinsic_calib
