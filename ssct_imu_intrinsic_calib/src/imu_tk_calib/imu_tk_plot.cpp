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

#include "ssct_imu_intrinsic_calib/imu_tk_calib/imu_tk_plot.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

#include <opencv2/imgproc.hpp>

namespace ssct_imu_intrinsic_calib
{

void ImuTkPlot::set_samples(const std::vector<Eigen::Vector3d> & accel)
{
  accel_ = accel;
}

void ImuTkPlot::set_intervals(const std::vector<std::pair<int, int>> & intervals)
{
  intervals_ = intervals;
}

void ImuTkPlot::set_threshold_multiplier(int threshold_multiplier)
{
  threshold_multiplier_ = threshold_multiplier;
}

void ImuTkPlot::clear_data()
{
  accel_.clear();
  intervals_.clear();
  threshold_multiplier_ = 0;
}

// Accel curves with the detected static intervals, in the spirit of the
// imu_tk gnuplot visualization (shaded regions plus an interval step line).
cv::Mat ImuTkPlot::draw() const
{
  constexpr int kWidth = 1200;
  constexpr int kHeight = 600;
  constexpr int kLeft = 70;
  constexpr int kRight = 30;
  constexpr int kTop = 60;
  constexpr int kBottom = 50;
  cv::Mat image(kHeight, kWidth, CV_8UC3, cv::Scalar(255, 255, 255));
  std::ostringstream title;
  title << "IMU-TK static intervals: " << intervals_.size() << " / "
        << accel_.size() << " samples";
  if (threshold_multiplier_ > 0) {
    title << ", threshold x" << threshold_multiplier_;
  }
  cv::putText(
    image, title.str(), cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6,
    cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
  if (accel_.empty()) {
    return image;
  }
  const cv::Rect plot(
    kLeft, kTop, kWidth - kLeft - kRight, kHeight - kTop - kBottom);
  cv::rectangle(image, plot, cv::Scalar(0, 0, 0), 1);
  double min_y = std::numeric_limits<double>::infinity();
  double max_y = -min_y;
  for (const auto & value : accel_) {
    for (int axis = 0; axis < 3; axis++) {
      min_y = std::min(min_y, value[axis]);
      max_y = std::max(max_y, value[axis]);
    }
  }
  if (!std::isfinite(min_y) || !std::isfinite(max_y)) {
    min_y = -1.0;
    max_y = 1.0;
  }
  if (max_y - min_y < 1e-9) {
    min_y -= 0.5;
    max_y += 0.5;
  }
  const double padding = 0.05 * (max_y - min_y);
  min_y -= padding;
  max_y += padding;

  const int n = static_cast<int>(accel_.size());
  const double span_x = std::max(1, n - 1);
  const auto map_x = [&](int index) {
      return plot.x + static_cast<int>(
        std::lround(index / span_x * static_cast<double>(plot.width)));
    };
  for (const auto & interval : intervals_) {
    const int x0 = map_x(interval.first);
    const int x1 = map_x(std::min(interval.second, n - 1));
    cv::rectangle(
      image, cv::Rect(cv::Point(x0, plot.y + 1), cv::Point(std::max(x1, x0 + 1),
      plot.y + plot.height - 1)), cv::Scalar(232, 232, 232), cv::FILLED);
  }
  const cv::Scalar colors[3] = {
    cv::Scalar(0, 0, 255), cv::Scalar(0, 160, 0), cv::Scalar(255, 0, 0)};
  const char * axis_names[3] = {"x", "y", "z"};
  const int stride = std::max(1, n / (2 * plot.width));
  for (int axis = 0; axis < 3; axis++) {
    cv::Point previous;
    bool has_previous = false;
    for (int i = 0; i < n; i += stride) {
      const double y = plot.y + plot.height -
        (accel_[i][axis] - min_y) / (max_y - min_y) * plot.height;
      const cv::Point point(map_x(i), static_cast<int>(std::lround(y)));
      if (has_previous) {
        cv::line(image, previous, point, colors[axis], 2, cv::LINE_AA);
      }
      previous = point;
      has_previous = true;
    }
    cv::putText(
      image, axis_names[axis], cv::Point(plot.x + plot.width - 25 + 8 * axis, plot.y - 10),
      cv::FONT_HERSHEY_SIMPLEX, 0.5, colors[axis], 1, cv::LINE_AA);
  }
  // Same interval indicator as imu_tk: a square wave whose top sits halfway
  // between the mean and the maximum accelerometer value.
  double sum = 0.0;
  double max_component = -std::numeric_limits<double>::infinity();
  for (const auto & value : accel_) {
    for (int axis = 0; axis < 3; axis++) {
      sum += value[axis];
      max_component = std::max(max_component, value[axis]);
    }
  }
  const double mean = sum / (3.0 * n);
  const double step_h = mean + (max_component - mean) / 2.0;
  const int step_y = plot.y + plot.height - static_cast<int>(std::lround(
      (step_h - min_y) / (max_y - min_y) * plot.height));
  const int base_y = plot.y + plot.height - 1;
  const cv::Scalar interval_color(255, 0, 255);
  for (const auto & interval : intervals_) {
    const int x0 = map_x(interval.first);
    const int x1 = map_x(std::min(interval.second, n - 1));
    cv::line(image, cv::Point(x0, base_y), cv::Point(x0, step_y),
      interval_color, 2, cv::LINE_AA);
    cv::line(image, cv::Point(x0, step_y), cv::Point(x1, step_y),
      interval_color, 2, cv::LINE_AA);
    cv::line(image, cv::Point(x1, step_y), cv::Point(x1, base_y),
      interval_color, 2, cv::LINE_AA);
  }
  std::ostringstream y_range;
  y_range << max_y;
  cv::putText(
    image, y_range.str(), cv::Point(5, plot.y + 12), cv::FONT_HERSHEY_SIMPLEX,
    0.4, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
  std::ostringstream y_min;
  y_min << min_y;
  cv::putText(
    image, y_min.str(), cv::Point(5, plot.y + plot.height), cv::FONT_HERSHEY_SIMPLEX,
    0.4, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
  cv::putText(
    image, "sample index", cv::Point(kWidth / 2 - 40, kHeight - 15),
    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
  return image;
}

}  // namespace ssct_imu_intrinsic_calib
