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

#include "ssct_imu_intrinsic_calib/allan_calib/allan_plot.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <sstream>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace ssct_imu_intrinsic_calib
{
namespace
{

constexpr int kPlotWidth = 1200;
constexpr int kPlotHeight = 760;

std::string decade_label(int exponent)
{
  std::ostringstream stream;
  stream << "1e" << exponent;
  return stream.str();
}

void draw_allan_row(
  cv::Mat & image, const std::array<std::vector<double>, 3> & tau,
  const std::array<std::vector<double>, 3> & adev, const std::string & title,
  int row, double min_tau, double max_tau)
{
  const int left = 90;
  const int right = 30;
  const int top = 60;
  const int bottom = 55;
  const int gap_x = 25;
  const int gap_y = 35;
  const int cell_width = (kPlotWidth - left - right - 2 * gap_x) / 3;
  const int cell_height = (kPlotHeight - top - bottom - gap_y) / 2;
  double min_y = std::numeric_limits<double>::infinity();
  double max_y = -min_y;
  for (const auto & values : adev) {
    for (const double value : values) {
      if (value > 0.0) {
        const double log_value = std::log10(value);
        min_y = std::min(min_y, log_value);
        max_y = std::max(max_y, log_value);
      }
    }
  }
  if (!std::isfinite(min_y)) {
    min_y = -6.0;
    max_y = -3.0;
  }
  if (max_y - min_y < 1e-6) {
    min_y -= 0.5;
    max_y += 0.5;
  }
  const double y_padding = 0.1 * (max_y - min_y);
  min_y -= y_padding;
  max_y += y_padding;
  const double x_min = std::log10(std::max(min_tau, 1e-12));
  double x_max = std::log10(std::max(max_tau, min_tau * 10.0));
  if (x_max - x_min < 1e-6) {
    x_max = x_min + 1.0;
  }
  const cv::Scalar colors[3] = {
    cv::Scalar(0, 0, 255), cv::Scalar(0, 160, 0), cv::Scalar(255, 0, 0)};
  const char * axis_names[3] = {"x", "y", "z"};
  for (int col = 0; col < 3; col++) {
    const int origin_x = left + col * (cell_width + gap_x);
    const int origin_y = top + row * (cell_height + gap_y);
    const cv::Rect plot(origin_x + 40, origin_y + 30, cell_width - 55, cell_height - 55);
    cv::rectangle(image, plot, cv::Scalar(0, 0, 0), 1);
    for (int exponent = static_cast<int>(std::ceil(x_min));
      exponent <= static_cast<int>(std::floor(x_max)); exponent++)
    {
      const double x = plot.x +
        (exponent - x_min) / (x_max - x_min) * plot.width;
      cv::line(
        image, cv::Point(x, plot.y), cv::Point(x, plot.y + plot.height),
        cv::Scalar(225, 225, 225), 1);
      if (row == 1) {
        cv::putText(
          image, decade_label(exponent), cv::Point(x - 12, plot.y + plot.height + 18),
          cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
      }
    }
    for (int exponent = static_cast<int>(std::ceil(min_y));
      exponent <= static_cast<int>(std::floor(max_y)); exponent++)
    {
      const double y = plot.y + plot.height -
        (exponent - min_y) / (max_y - min_y) * plot.height;
      cv::line(
        image, cv::Point(plot.x, y), cv::Point(plot.x + plot.width, y),
        cv::Scalar(225, 225, 225), 1);
      if (col == 0) {
        cv::putText(
          image, decade_label(exponent), cv::Point(origin_x + 2, y + 5),
          cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
      }
    }
    cv::Point previous;
    bool has_previous = false;
    const size_t count = std::min(tau[col].size(), adev[col].size());
    for (size_t i = 0; i < count; i++) {
      if (tau[col][i] <= 0.0 || adev[col][i] <= 0.0) {
        continue;
      }
      const double x = plot.x +
        (std::log10(tau[col][i]) - x_min) / (x_max - x_min) * plot.width;
      const double y = plot.y + plot.height -
        (std::log10(adev[col][i]) - min_y) / (max_y - min_y) * plot.height;
      const cv::Point point(x, y);
      if (has_previous) {
        cv::line(image, previous, point, colors[col], 2, cv::LINE_AA);
      }
      previous = point;
      has_previous = true;
    }
    cv::putText(
      image, title + " " + axis_names[col], cv::Point(origin_x + 40, origin_y + 20),
      cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
  }
  cv::putText(
    image, title + " ADEV",
    cv::Point(5, top + row * (cell_height + gap_y) + cell_height / 2),
    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
}

}  // namespace

void AllanPlot::set_accel_curve(
  int axis, const std::vector<double> & tau, const std::vector<double> & adev)
{
  if (axis < 0 || axis >= 3) {
    return;
  }
  accel_tau_[axis] = tau;
  accel_adev_[axis] = adev;
}

void AllanPlot::set_gyro_curve(
  int axis, const std::vector<double> & tau, const std::vector<double> & adev)
{
  if (axis < 0 || axis >= 3) {
    return;
  }
  gyro_tau_[axis] = tau;
  gyro_adev_[axis] = adev;
}

void AllanPlot::set_sample_count(int sample_count)
{
  sample_count_ = sample_count;
}

void AllanPlot::set_sample_rate(double sample_rate)
{
  sample_rate_ = sample_rate;
}

void AllanPlot::clear_data()
{
  accel_tau_ = {};
  accel_adev_ = {};
  gyro_tau_ = {};
  gyro_adev_ = {};
  sample_count_ = 0;
  sample_rate_ = 0.0;
}

cv::Mat AllanPlot::draw() const
{
  cv::Mat image(kPlotHeight, kPlotWidth, CV_8UC3, cv::Scalar(255, 255, 255));
  double min_tau = std::numeric_limits<double>::infinity();
  double max_tau = 0.0;
  for (const auto * curves : {&gyro_tau_, &accel_tau_}) {
    for (const auto & values : *curves) {
      for (const double tau : values) {
        if (tau > 0.0) {
          min_tau = std::min(min_tau, tau);
          max_tau = std::max(max_tau, tau);
        }
      }
    }
  }
  if (!std::isfinite(min_tau) || max_tau <= 0.0) {
    min_tau = 1.0;
    max_tau = 1000.0;
  }
  std::ostringstream title;
  title  << "IMU Allan deviation: " << sample_count_ << " samples, "
         << sample_rate_ << " Hz";
  cv::putText(
    image, title.str(), cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.65,
    cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
  draw_allan_row(image, gyro_tau_, gyro_adev_, "gyro", 0, min_tau, max_tau);
  draw_allan_row(image, accel_tau_, accel_adev_, "accel", 1, min_tau, max_tau);
  cv::putText(
    image, "tau [s]", cv::Point(kPlotWidth / 2 - 30, kPlotHeight - 10),
    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
  return image;
}

}  // namespace ssct_imu_intrinsic_calib
