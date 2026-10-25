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
#include "ssct_camera_intrinsic_calib/calibrator_interface.hpp"

namespace ssct_camera_intrinsic_calib
{

// ftheta_kb (Kannala-Brandt) fisheye camera model, calibrated by cv::fisheye::calibrate
//   intrinsics: [fx, fy, cx, cy]
//   distortion: [k1, k2, k3, k4]
class OpenCVFisheyeCalibrator : public CalibratorInterface
{
public:
  void set_image_size(const cv::Size & image_size) override;
  void add_points(
    const std::vector<cv::Point2f> & image_points,
    const std::vector<cv::Point3f> & object_points) override;
  bool optimize() override;
  void clear() override;
  // get result
  std::string get_type() const override;
  std::vector<double> get_intrinsics() const override;
  std::vector<double> get_distortion_coeffs() const override;
  cv::Size get_image_size() const override;
  const std::string & get_status_message() const override;

private:
  cv::Size image_size_;
  std::vector<std::vector<cv::Point2f>> image_points_;
  std::vector<std::vector<cv::Point3f>> object_points_;
  bool calibrated_{false};
  cv::Mat camera_intrinsic_;
  cv::Mat camera_distortion_;
  std::vector<cv::Vec3d> board_rotations_;
  std::vector<cv::Vec3d> board_translations_;
  std::string status_message_;
};

}  // namespace ssct_camera_intrinsic_calib
