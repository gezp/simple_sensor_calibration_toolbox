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

#include "ssct_camera_intrinsic_calib/detector/chessboard_detector.hpp"

namespace ssct_camera_intrinsic_calib
{

ChessboardDetector::ChessboardDetector(const YAML::Node & config)
{
  int corner_cols = config["chessboard"]["cols"].as<int>();
  int corner_rows = config["chessboard"]["rows"].as<int>();
  corner_size_ = cv::Size(corner_cols, corner_rows);
  float grid_size = config["chessboard"]["size"].as<float>();
  // generate 3D corner points
  for (int i = 0; i < corner_rows; i++) {
    for (int j = 0; j < corner_cols; j++) {
      pattern_points_.push_back(cv::Point3f(j * grid_size, i * grid_size, 0.0f));
    }
  }
}

bool ChessboardDetector::detect(
  const cv::Mat & image, std::vector<cv::Point2f> & corners) const
{
  cv::Mat gray_image;
  cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
  if (!cv::findChessboardCorners(gray_image, corner_size_, corners)) {
    return false;
  }
  cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::EPS, 50, 0.001);
  cv::cornerSubPix(gray_image, corners, cv::Size(5, 5), cv::Size(-1, -1), criteria);
  return true;
}

const cv::Size & ChessboardDetector::get_corner_size() const
{
  return corner_size_;
}

const std::vector<cv::Point3f> & ChessboardDetector::get_pattern_points() const
{
  return pattern_points_;
}

}  // namespace ssct_camera_intrinsic_calib
