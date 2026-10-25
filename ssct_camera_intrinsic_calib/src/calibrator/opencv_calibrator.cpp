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

#include "ssct_camera_intrinsic_calib/calibrator/opencv_calibrator.hpp"

namespace ssct_camera_intrinsic_calib
{

void OpenCVCalibrator::set_image_size(const cv::Size & image_size)
{
  image_size_ = image_size;
}

void OpenCVCalibrator::add_points(
  const std::vector<cv::Point2f> & image_points,
  const std::vector<cv::Point3f> & object_points)
{
  image_points_.push_back(image_points);
  object_points_.push_back(object_points);
}

bool OpenCVCalibrator::optimize()
{
  if (image_points_.empty()) {
    status_message_ = "[optimize] failed to optimize, no image points.";
    return false;
  }
  double rms = cv::calibrateCamera(
    object_points_, image_points_, image_size_, camera_intrinsic_, camera_distortion_,
    board_rotations_, board_translations_);
  calibrated_ = true;
  status_message_ = "[optimize] successed to optimize, image:" +
    std::to_string(image_points_.size()) +
    ", RMS re-projection error:" + std::to_string(rms);
  return true;
}

void OpenCVCalibrator::clear()
{
  image_size_ = cv::Size(0, 0);
  image_points_.clear();
  object_points_.clear();
  calibrated_ = false;
  status_message_ = "";
}

std::string OpenCVCalibrator::get_type() const
{
  return "pinhole_radtan";
}

std::vector<double> OpenCVCalibrator::get_intrinsics() const
{
  if (!calibrated_) {
    return {};
  }
  return {
    camera_intrinsic_.at<double>(0, 0), camera_intrinsic_.at<double>(1, 1),
    camera_intrinsic_.at<double>(0, 2), camera_intrinsic_.at<double>(1, 2)};
}

std::vector<double> OpenCVCalibrator::get_distortion_coeffs() const
{
  if (!calibrated_) {
    return {};
  }
  cv::Mat distortion = camera_distortion_.reshape(1, 1);
  std::vector<double> coeffs(distortion.begin<double>(), distortion.end<double>());
  coeffs.resize(5, 0.0);
  return coeffs;
}

cv::Size OpenCVCalibrator::get_image_size() const
{
  return image_size_;
}

const std::string & OpenCVCalibrator::get_status_message() const
{
  return status_message_;
}

}  // namespace ssct_camera_intrinsic_calib
