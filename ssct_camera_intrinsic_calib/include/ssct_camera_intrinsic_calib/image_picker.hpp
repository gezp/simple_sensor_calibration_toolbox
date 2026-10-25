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

#include <array>
#include <cstdint>
#include <vector>

#include "opencv2/opencv.hpp"

namespace ssct_camera_intrinsic_calib
{

struct ImagePickerInfo
{
  int rows{3};
  int cols{3};
  // Raw coverage percentage of each grid cell, row-major.
  std::vector<int> cell_coverage_pct;
  std::vector<uint8_t> cell_passed;
  // Normalized coverage: 100 means the requirement is met.
  int size_coverage_pct{0};
  int tilt_coverage_pct{0};
  int valid_frames{0};
  int max_frames{60};
  bool metrics_ready{false};
};

class ImagePicker
{
public:
  explicit ImagePicker(const YAML::Node & config = YAML::Node());

  void set_image_size(const cv::Size & image_size);

  // Return true when the image is accepted into the calibration set.
  bool pick(
    const std::vector<cv::Point2f> & corners,
    const std::array<cv::Point2f, 4> & board_corners);

  bool ready() const;
  void reset();
  const ImagePickerInfo & get_info() const {return info_;}

  // Draw the coverage grid and metrics onto a copy of the image.
  cv::Mat draw_debug(const cv::Mat & image) const;

private:
  struct Sample
  {
    cv::Point2f center;
    std::array<double, 4> angles;
  };

  void init(const cv::Size & image_size);
  void update_info();

private:
  int rows_{3};
  int cols_{3};
  int mask_scale_{4};
  double min_cell_coverage_{0.30};
  double min_board_area_ratio_{0.04};
  double min_corner_angle_deg_{40.0};
  double max_corner_angle_deg_{140.0};
  double min_move_ratio_{0.08};
  double min_new_area_ratio_{0.005};
  double min_angle_change_deg_{5.0};
  int min_valid_image_{15};
  int max_valid_image_{60};
  double size_max_ratio_{0.20};
  double size_span_ratio_{0.10};
  double tilt_span_deg_{15.0};

  bool initialized_{false};
  cv::Size image_size_{0, 0};
  cv::Mat mask_;
  std::vector<int> cell_pixels_;
  std::vector<int> cell_covered_;

  std::vector<Sample> samples_;
  int valid_count_{0};
  bool has_metrics_{false};
  double min_area_ratio_{0.0};
  double max_area_ratio_{0.0};
  double min_skew_deg_{0.0};
  double max_skew_deg_{0.0};

  ImagePickerInfo info_;
};

}  // namespace ssct_camera_intrinsic_calib
