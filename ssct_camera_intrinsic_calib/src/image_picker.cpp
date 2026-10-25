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

#include "ssct_camera_intrinsic_calib/image_picker.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <string>

namespace ssct_camera_intrinsic_calib
{

namespace
{

template<typename T>
T get_config(const YAML::Node & config, const char * key, const T & default_value)
{
  if (!config || !config[key]) {
    return default_value;
  }
  return config[key].as<T>();
}

double corner_angle(const cv::Point2f & prev, const cv::Point2f & corner, const cv::Point2f & next)
{
  const cv::Point2f a = prev - corner;
  const cv::Point2f b = next - corner;
  const double norm_a = cv::norm(a);
  const double norm_b = cv::norm(b);
  if (norm_a < 1e-9 || norm_b < 1e-9) {
    return 90.0;
  }
  const double cosine = a.dot(b) / (norm_a * norm_b);
  return std::acos(std::clamp(cosine, -1.0, 1.0)) * 180.0 / CV_PI;
}

int coverage_percent(double ratio)
{
  return static_cast<int>(std::lround(std::clamp(ratio, 0.0, 1.0) * 100.0));
}

int normalized_percent(double actual, double required)
{
  if (required <= 0.0) {
    return 100;
  }
  return static_cast<int>(std::floor(std::clamp(actual / required, 0.0, 1.0) * 100.0));
}

}  // namespace

ImagePicker::ImagePicker(const YAML::Node & config)
{
  rows_ = std::max(1, get_config<int>(config, "rows", 3));
  cols_ = std::max(1, get_config<int>(config, "cols", 3));
  mask_scale_ = std::max(1, get_config<int>(config, "mask_scale", 4));
  min_cell_coverage_ = std::clamp(get_config<double>(config, "min_cell_coverage", 0.30), 0.0, 1.0);
  min_board_area_ratio_ = get_config<double>(config, "min_board_area_ratio", 0.04);
  min_corner_angle_deg_ = get_config<double>(config, "min_corner_angle_deg", 40.0);
  max_corner_angle_deg_ = get_config<double>(config, "max_corner_angle_deg", 140.0);
  min_move_ratio_ = get_config<double>(config, "min_move_ratio", 0.08);
  min_new_area_ratio_ = get_config<double>(config, "min_new_area_ratio", 0.005);
  min_angle_change_deg_ = get_config<double>(config, "min_angle_change_deg", 5.0);
  min_valid_image_ = std::max(1, get_config<int>(config, "min_valid_image", 15));
  max_valid_image_ = std::max(1, get_config<int>(config, "max_valid_image", 60));
  size_max_ratio_ = std::max(1e-6, get_config<double>(config, "size_max_ratio", 0.20));
  size_span_ratio_ = std::max(1e-6, get_config<double>(config, "size_span_ratio", 0.10));
  tilt_span_deg_ = std::max(1e-6, get_config<double>(config, "tilt_span_deg", 15.0));
  reset();
}

void ImagePicker::reset()
{
  initialized_ = false;
  image_size_ = cv::Size(0, 0);
  mask_.release();
  cell_pixels_.assign(static_cast<size_t>(rows_ * cols_), 0);
  cell_covered_.assign(static_cast<size_t>(rows_ * cols_), 0);
  samples_.clear();
  valid_count_ = 0;
  has_metrics_ = false;
  min_area_ratio_ = 0.0;
  max_area_ratio_ = 0.0;
  min_skew_deg_ = 0.0;
  max_skew_deg_ = 0.0;

  info_ = ImagePickerInfo();
  info_.rows = rows_;
  info_.cols = cols_;
  info_.cell_coverage_pct.assign(static_cast<size_t>(rows_ * cols_), 0);
  info_.cell_passed.assign(static_cast<size_t>(rows_ * cols_), 0);
  info_.max_frames = max_valid_image_;
}

void ImagePicker::init(const cv::Size & image_size)
{
  initialized_ = true;
  image_size_ = image_size;
  const int mask_width = std::max(1, (image_size.width + mask_scale_ - 1) / mask_scale_);
  const int mask_height = std::max(1, (image_size.height + mask_scale_ - 1) / mask_scale_);
  mask_ = cv::Mat::zeros(mask_height, mask_width, CV_8UC1);

  cell_pixels_.assign(static_cast<size_t>(rows_ * cols_), 0);
  cell_covered_.assign(static_cast<size_t>(rows_ * cols_), 0);
  for (int y = 0; y < mask_.rows; ++y) {
    for (int x = 0; x < mask_.cols; ++x) {
      const int row = y * rows_ / mask_.rows;
      const int col = x * cols_ / mask_.cols;
      ++cell_pixels_[static_cast<size_t>(row * cols_ + col)];
    }
  }

  samples_.clear();
  valid_count_ = 0;
  has_metrics_ = false;
  min_area_ratio_ = 0.0;
  max_area_ratio_ = 0.0;
  min_skew_deg_ = 0.0;
  max_skew_deg_ = 0.0;
  update_info();
}

void ImagePicker::set_image_size(const cv::Size & image_size)
{
  if (!initialized_ || image_size_ != image_size) {
    init(image_size);
  }
}

bool ImagePicker::pick(
  const std::vector<cv::Point2f> & corners,
  const std::array<cv::Point2f, 4> & board_corners)
{
  if (corners.empty()) {
    return false;
  }
  const std::array<double, 4> angles = {
    corner_angle(board_corners[3], board_corners[0], board_corners[1]),
    corner_angle(board_corners[0], board_corners[1], board_corners[2]),
    corner_angle(board_corners[1], board_corners[2], board_corners[3]),
    corner_angle(board_corners[2], board_corners[3], board_corners[0])};
  for (const double angle : angles) {
    if (angle < min_corner_angle_deg_ || angle > max_corner_angle_deg_) {
      return false;
    }
  }

  std::vector<cv::Point2f> hull;
  cv::convexHull(corners, hull);
  const double board_area = cv::contourArea(hull);
  const double image_area = static_cast<double>(image_size_.width) * image_size_.height;
  const double area_ratio = board_area / image_area;
  if (area_ratio < min_board_area_ratio_) {
    return false;
  }

  const cv::Point2f center(
    (board_corners[0].x + board_corners[1].x + board_corners[2].x + board_corners[3].x) / 4.0f,
    (board_corners[0].y + board_corners[1].y + board_corners[2].y + board_corners[3].y) / 4.0f);
  const double skew_deg =
    (std::abs(angles[0] - 90.0) + std::abs(angles[1] - 90.0) +
    std::abs(angles[2] - 90.0) + std::abs(angles[3] - 90.0)) /
    4.0;

  std::vector<cv::Point> scaled_hull;
  scaled_hull.reserve(hull.size());
  for (const auto & point : hull) {
    const int x = std::clamp(cvRound(point.x) / mask_scale_, 0, mask_.cols - 1);
    const int y = std::clamp(cvRound(point.y) / mask_scale_, 0, mask_.rows - 1);
    scaled_hull.emplace_back(x, y);
  }
  cv::Mat candidate = cv::Mat::zeros(mask_.size(), CV_8UC1);
  cv::fillConvexPoly(candidate, scaled_hull, cv::Scalar(1), cv::LINE_8);
  cv::Mat new_mask;
  cv::subtract(candidate, mask_, new_mask);
  const int new_pixels = cv::countNonZero(new_mask);
  const double new_area_ratio = static_cast<double>(new_pixels) / mask_.total();

  bool accept = valid_count_ == 0 || new_area_ratio >= min_new_area_ratio_;
  if (!accept) {
    const double move_dist =
      min_move_ratio_ * (image_size_.width + image_size_.height) / 2.0;
    bool moved = false;
    bool similar_pose = false;
    for (const auto & sample : samples_) {
      const double dist = std::hypot(center.x - sample.center.x, center.y - sample.center.y);
      if (dist >= move_dist) {
        moved = true;
        continue;
      }
      bool all_angles_similar = true;
      for (size_t i = 0; i < angles.size(); ++i) {
        if (std::abs(angles[i] - sample.angles[i]) >= min_angle_change_deg_) {
          all_angles_similar = false;
          break;
        }
      }
      if (all_angles_similar) {
        similar_pose = true;
        break;
      }
    }
    accept = moved || !similar_pose;
  }
  if (valid_count_ >= max_valid_image_) {
    accept = false;
  }
  if (!accept) {
    return false;
  }

  for (int y = 0; y < mask_.rows; ++y) {
    for (int x = 0; x < mask_.cols; ++x) {
      if (new_mask.at<uchar>(y, x) == 0) {
        continue;
      }
      mask_.at<uchar>(y, x) = 1;
      const int row = y * rows_ / mask_.rows;
      const int col = x * cols_ / mask_.cols;
      ++cell_covered_[static_cast<size_t>(row * cols_ + col)];
    }
  }

  samples_.push_back({center, angles});
  ++valid_count_;
  if (!has_metrics_) {
    min_area_ratio_ = area_ratio;
    max_area_ratio_ = area_ratio;
    min_skew_deg_ = skew_deg;
    max_skew_deg_ = skew_deg;
    has_metrics_ = true;
  } else {
    min_area_ratio_ = std::min(min_area_ratio_, area_ratio);
    max_area_ratio_ = std::max(max_area_ratio_, area_ratio);
    min_skew_deg_ = std::min(min_skew_deg_, skew_deg);
    max_skew_deg_ = std::max(max_skew_deg_, skew_deg);
  }
  update_info();
  return true;
}

void ImagePicker::update_info()
{
  const size_t cell_count = static_cast<size_t>(rows_ * cols_);
  info_.rows = rows_;
  info_.cols = cols_;
  info_.cell_coverage_pct.assign(cell_count, 0);
  info_.cell_passed.assign(cell_count, 0);
  const int target_cell_pct = coverage_percent(min_cell_coverage_);
  for (size_t i = 0; i < cell_count; ++i) {
    const double ratio =
      cell_pixels_[i] > 0 ? static_cast<double>(cell_covered_[i]) / cell_pixels_[i] : 0.0;
    const int percent = coverage_percent(ratio);
    info_.cell_coverage_pct[i] = percent;
    info_.cell_passed[i] = percent >= target_cell_pct ? 1 : 0;
  }

  info_.valid_frames = valid_count_;
  info_.max_frames = max_valid_image_;
  info_.size_coverage_pct = 0;
  info_.tilt_coverage_pct = 0;
  if (has_metrics_) {
    const double size_span = max_area_ratio_ - min_area_ratio_;
    const double size_score = std::min(
      {1.0, max_area_ratio_ / size_max_ratio_, size_span / size_span_ratio_});
    info_.size_coverage_pct = normalized_percent(size_score, 1.0);

    const double tilt_span = max_skew_deg_ - min_skew_deg_;
    info_.tilt_coverage_pct = normalized_percent(tilt_span, tilt_span_deg_);
  }
  const bool all_cells_passed =
    std::all_of(info_.cell_passed.begin(), info_.cell_passed.end(), [](uint8_t passed) {
        return passed != 0;
    });
  info_.metrics_ready =
    all_cells_passed && info_.size_coverage_pct >= 100 && info_.tilt_coverage_pct >= 100;
}

bool ImagePicker::ready() const
{
  if (!initialized_ || valid_count_ < min_valid_image_) {
    return false;
  }
  // The frame cap is a hard stop, otherwise auto calibration would keep
  // waiting forever when a region cannot be covered (for example fisheye edges).
  return info_.metrics_ready || valid_count_ >= max_valid_image_;
}

cv::Mat ImagePicker::draw_debug(const cv::Mat & image) const
{
  if (image.empty()) {
    return image.clone();
  }
  const int side_width = std::max(200, static_cast<int>(image.rows * 0.28));
  cv::Mat canvas(
    image.rows, image.cols + side_width, image.type(), cv::Scalar(28, 32, 38));
  image.copyTo(canvas(cv::Rect(0, 0, image.cols, image.rows)));

  const cv::Scalar green(80, 190, 80);
  const cv::Scalar red(60, 60, 220);
  const cv::Scalar grid_color(235, 235, 235);
  const cv::Scalar label_color(175, 175, 175);
  const cv::Scalar white(245, 245, 245);

  const double cell_font = std::max(0.9, 1.1 * image.rows / 720.0);
  const int cell_thickness = std::max(1, static_cast<int>(std::lround(2.0 * image.rows / 720.0)));
  for (int row = 0; row < info_.rows; ++row) {
    for (int col = 0; col < info_.cols; ++col) {
      const int x0 = col * image.cols / info_.cols;
      const int x1 = (col + 1) * image.cols / info_.cols;
      const int y0 = row * image.rows / info_.rows;
      const int y1 = (row + 1) * image.rows / info_.rows;
      const size_t index = static_cast<size_t>(row * info_.cols + col);
      const bool passed = index < info_.cell_passed.size() && info_.cell_passed[index] != 0;

      const int cell_percent =
        index < info_.cell_coverage_pct.size() ? info_.cell_coverage_pct[index] : 0;
      const std::string text = std::to_string(cell_percent) + "%";
      int baseline = 0;
      const cv::Size text_size =
        cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, cell_font, cell_thickness, &baseline);
      const cv::Point origin(
        x0 + (x1 - x0 - text_size.width) / 2, y0 + (y1 - y0 + text_size.height) / 2);
      cv::putText(
        canvas, text, origin, cv::FONT_HERSHEY_SIMPLEX, cell_font, cv::Scalar(0, 0, 0),
        cell_thickness + 2, cv::LINE_AA);
      cv::putText(
        canvas, text, origin, cv::FONT_HERSHEY_SIMPLEX, cell_font, passed ? green : red,
        cell_thickness, cv::LINE_AA);
    }
  }
  for (int col = 1; col < info_.cols; ++col) {
    const int x = col * image.cols / info_.cols;
    cv::line(canvas, cv::Point(x, 0), cv::Point(x, image.rows - 1), grid_color, 1, cv::LINE_AA);
  }
  for (int row = 1; row < info_.rows; ++row) {
    const int y = row * image.rows / info_.rows;
    cv::line(canvas, cv::Point(0, y), cv::Point(image.cols - 1, y), grid_color, 1, cv::LINE_AA);
  }
  cv::line(
    canvas, cv::Point(image.cols, 0), cv::Point(image.cols, image.rows - 1),
    cv::Scalar(70, 75, 80), 1);

  const double side_font = std::max(0.5, 0.65 * image.rows / 720.0);
  const int side_thickness = std::max(1, static_cast<int>(std::lround(2.0 * image.rows / 720.0)));
  const int pad = std::max(14, static_cast<int>(image.rows * 0.04));
  const int line_height = std::max(24, static_cast<int>(image.rows * 0.06));
  const int x = image.cols + pad;
  int y = pad + static_cast<int>(image.rows * 0.05);
  auto draw_line = [&](const std::string & text, const cv::Scalar & color) {
      cv::putText(
      canvas, text, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, side_font, color,
      side_thickness, cv::LINE_AA);
      y += line_height;
    };

  draw_line("FRAMES " + std::to_string(info_.valid_frames) + "/" + std::to_string(info_.max_frames),
    white);
  y += line_height / 2;
  draw_line("SIZE COVERAGE", label_color);
  draw_line(
    std::to_string(info_.size_coverage_pct) + "%",
    info_.size_coverage_pct >= 100 ? green : red);
  y += line_height / 2;
  draw_line("TILT COVERAGE", label_color);
  draw_line(
    std::to_string(info_.tilt_coverage_pct) + "%",
    info_.tilt_coverage_pct >= 100 ? green : red);
  return canvas;
}

}  // namespace ssct_camera_intrinsic_calib
