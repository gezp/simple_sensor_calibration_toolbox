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

#include "ssct_imu_intrinsic_calib/imu_tk_calib/static_pose_collector.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>
#include <vector>

#include "ssct_imu_intrinsic_calib/utils.hpp"

namespace ssct_imu_intrinsic_calib
{
namespace
{

constexpr double kMinPoseAngleDegrees = 15.0;
constexpr double kNoStaticTimeoutFactor = 3.0;
constexpr double kMotionVarianceFactor = 4.0;

double median(std::vector<double> values)
{
  const size_t middle = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + middle, values.end());
  return values[middle];
}

Eigen::Vector3d window_variance(
  const std::vector<Eigen::Vector3d> & sums,
  const std::vector<Eigen::Vector3d> & squares, int start, int end)
{
  const int count = end - start + 1;
  const Eigen::Vector3d sum = sums[end + 1] - sums[start];
  const Eigen::Vector3d square_sum = squares[end + 1] - squares[start];
  const Eigen::Vector3d variance =
    (square_sum - sum.cwiseProduct(sum) / static_cast<double>(count)) /
    static_cast<double>(count - 1);
  return variance.cwiseMax(0.0);
}

std::vector<StaticInterval> detect_static_intervals(
  const std::vector<Eigen::Vector3d> & sums,
  const std::vector<Eigen::Vector3d> & squares, int n, double threshold, int window)
{
  std::vector<StaticInterval> intervals;
  const int half = window / 2;
  bool in_static = false;
  StaticInterval current{0, 0};
  for (int i = half; i < n - half; i++) {
    const double variance_norm =
      window_variance(sums, squares, i - half, i + half).norm();
    if (!in_static && variance_norm < threshold) {
      current.start = i;
      in_static = true;
    } else if (in_static && variance_norm >= threshold) {
      current.end = i - 1;
      in_static = false;
      intervals.push_back(current);
    }
  }
  if (in_static) {
    current.end = n - half - 1;
    intervals.push_back(current);
  }
  return intervals;
}

std::vector<StaticInterval> filter_intervals(
  const std::vector<StaticInterval> & intervals, int min_samples)
{
  std::vector<StaticInterval> filtered;
  for (const auto & interval : intervals) {
    if (interval.end - interval.start + 1 >= min_samples) {
      filtered.push_back(interval);
    }
  }
  return filtered;
}

// The sensor bias is unknown, especially for raw accelerometer counts. The
// midrange of the static means approximates it without assuming uniform poses.
Eigen::Vector3d midrange_bias(const std::vector<Eigen::Vector3d> & means)
{
  Eigen::Vector3d min_value = Eigen::Vector3d::Constant(
    std::numeric_limits<double>::max());
  Eigen::Vector3d max_value = Eigen::Vector3d::Constant(
    -std::numeric_limits<double>::max());
  for (const auto & mean : means) {
    min_value = min_value.cwiseMin(mean);
    max_value = max_value.cwiseMax(mean);
  }
  return 0.5 * (min_value + max_value);
}

double direction_angle(const Eigen::Vector3d & first, const Eigen::Vector3d & second)
{
  const double norms = first.norm() * second.norm();
  if (!(norms > 0.0)) {
    return 0.0;
  }
  return std::acos(std::clamp(first.dot(second) / norms, -1.0, 1.0));
}

std::vector<StaticInterval> filter_distinct_intervals(
  const std::vector<ssct_common::ImuData> & samples,
  const std::vector<StaticInterval> & intervals, double min_angle)
{
  if (intervals.empty()) {
    return {};
  }
  std::vector<Eigen::Vector3d> means;
  means.reserve(intervals.size());
  for (const auto & interval : intervals) {
    means.push_back(interval_accel_mean(samples, interval.start, interval.end));
  }
  const Eigen::Vector3d bias = midrange_bias(means);
  std::vector<StaticInterval> distinct;
  std::vector<Eigen::Vector3d> accepted_directions;
  for (size_t i = 0; i < intervals.size(); i++) {
    const Eigen::Vector3d direction = means[i] - bias;
    bool is_distinct = true;
    for (const auto & other : accepted_directions) {
      if (direction_angle(direction, other) < min_angle) {
        is_distinct = false;
        break;
      }
    }
    // The first interval is accepted by definition; with a single pose the
    // gravity direction cannot be distinguished from the sensor bias.
    if (is_distinct) {
      distinct.push_back(intervals[i]);
      accepted_directions.push_back(direction);
    }
  }
  return distinct;
}

}  // namespace

StaticPoseCollector::StaticPoseCollector(
  double static_window, int interval_samples, int min_intervals)
: static_window_(static_window),
  interval_samples_(interval_samples),
  min_intervals_(min_intervals)
{
}

void StaticPoseCollector::reset()
{
  valid_ = false;
  message_.clear();
  sample_count_ = 0;
  sample_period_ = 0.0;
  candidates_.clear();
  display_intervals_.clear();
  static_tail_start_ = -1;
  tail_state_ = PoseTailState::kNoStatic;
  recent_moving_ = false;
  since_last_pose_seconds_ = 0.0;
}

void StaticPoseCollector::collect(
  const std::vector<ssct_common::ImuData> & samples)
{
  reset();
  const int n = static_cast<int>(samples.size());
  sample_count_ = n;
  if (n < 20 || interval_samples_ < 1 || min_intervals_ < 2) {
    message_ = "not enough samples or invalid TK configuration";
    return;
  }

  std::vector<double> deltas;
  deltas.reserve(samples.size());
  for (size_t i = 1; i < samples.size(); i++) {
    const double delta = samples[i].time - samples[i - 1].time;
    if (delta > 0.0) {
      deltas.push_back(delta);
    }
  }
  if (deltas.empty()) {
    message_ = "IMU timestamps are not strictly increasing";
    return;
  }
  const double dt = median(deltas);
  if (!(dt > 0.0)) {
    message_ = "invalid IMU sample period";
    return;
  }
  sample_period_ = dt;

  int window = static_cast<int>(std::llround(static_window_ / dt));
  window = std::max(window, 11);
  if (window % 2 == 0) {
    window++;
  }
  if (window * 2 >= n) {
    message_ = "recording is too short for static interval detection";
    return;
  }
  std::vector<Eigen::Vector3d> sums(n + 1, Eigen::Vector3d::Zero());
  std::vector<Eigen::Vector3d> squares(n + 1, Eigen::Vector3d::Zero());
  for (int i = 0; i < n; i++) {
    const Eigen::Vector3d & accel = samples[i].linear_acceleration;
    sums[i + 1] = sums[i] + accel;
    squares[i + 1] = squares[i] + accel.cwiseProduct(accel);
  }
  // Use the quietest window as noise reference so playback may start mid-motion.
  const int half = window / 2;
  double initial_variance = std::numeric_limits<double>::max();
  for (int i = half; i < n - half; i++) {
    initial_variance = std::min(
      initial_variance, window_variance(sums, squares, i - half, i + half).norm());
  }
  if (!(initial_variance > 0.0)) {
    message_ = "no static window found";
    return;
  }

  const double min_angle = kMinPoseAngleDegrees * std::acos(-1.0) / 180.0;
  bool tail_filtered = false;
  bool tail_accepted = false;
  for (int multiplier = 4; multiplier <= 12; multiplier++) {
    const auto detected = detect_static_intervals(
      sums, squares, n, multiplier * initial_variance, window);
    // Track whether the recording currently ends inside a static interval; this
    // drives the keep-still / move-now prompts. Any threshold that sees it counts.
    if (!detected.empty() && detected.back().end >= n - half - 2 &&
      (static_tail_start_ < 0 || detected.back().start < static_tail_start_))
    {
      static_tail_start_ = detected.back().start;
    }
    const auto filtered = filter_intervals(detected, interval_samples_);
    if (!filtered.empty() && filtered.back().end >= n - half - 2) {
      tail_filtered = true;
    }
    const auto intervals =
      filter_distinct_intervals(samples, filtered, min_angle);
    if (!intervals.empty() && intervals.back().end >= n - half - 2) {
      tail_accepted = true;
    }
    if (intervals.size() > display_intervals_.size()) {
      display_intervals_ = intervals;
    }
    if (intervals.empty()) {
      continue;
    }
    PoseCandidate candidate;
    candidate.threshold_multiplier = multiplier;
    candidate.intervals = intervals;
    candidates_.push_back(std::move(candidate));
  }
  valid_ = true;

  std::vector<Eigen::Vector3d> accepted_means;
  accepted_means.reserve(display_intervals_.size());
  int last_pose_end = -1;
  for (const auto & interval : display_intervals_) {
    accepted_means.push_back(interval_accel_mean(samples, interval.start, interval.end));
    last_pose_end = std::max(last_pose_end, interval.end);
  }
  Eigen::Vector3d pose_bias = Eigen::Vector3d::Zero();
  if (!accepted_means.empty()) {
    pose_bias = midrange_bias(accepted_means);
    since_last_pose_seconds_ =
      last_pose_end >= 0 ? (n - 1 - last_pose_end) * dt : 0.0;
  }

  const double hold_seconds = static_window_ + interval_samples_ * dt;
  if (static_tail_start_ >= 0) {
    const double held_seconds = (n - 1 - static_tail_start_) * dt;
    if (held_seconds < hold_seconds) {
      tail_state_ = PoseTailState::kHolding;
    } else if (tail_accepted) {
      tail_state_ = PoseTailState::kAccepted;
    } else {
      const StaticInterval tail{static_tail_start_, n - 1};
      const Eigen::Vector3d tail_direction =
        interval_accel_mean(samples, tail.start, tail.end) - pose_bias;
      double min_tail_angle = std::numeric_limits<double>::max();
      for (const auto & mean : accepted_means) {
        min_tail_angle = std::min(
          min_tail_angle, direction_angle(tail_direction, mean - pose_bias));
      }
      const bool duplicate = tail_filtered && !accepted_means.empty() &&
        min_tail_angle < min_angle;
      tail_state_ = duplicate ?
        PoseTailState::kDuplicate : PoseTailState::kInvalid;
    }
  }

  recent_moving_ =
    window_variance(sums, squares, n - window, n - 1).norm() >
    kMotionVarianceFactor * initial_variance;
}

std::string StaticPoseCollector::progress() const
{
  std::ostringstream stream;
  stream << "static poses: " << display_intervals_.size() << " / "
         << min_intervals_;
  return stream.str();
}

std::string StaticPoseCollector::hint() const
{
  if (!(sample_period_ > 0.0)) {
    return "hold still until the status says to move";
  }
  // A static interval keeps window/2 samples at each end, so a pose has to be
  // held this long before the detector can extract interval_samples from it.
  const double hold_seconds =
    static_window_ + interval_samples_ * sample_period_;
  const size_t pose_count = display_intervals_.size();
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(1);
  switch (tail_state_) {
    case PoseTailState::kHolding: {
        const double held_seconds = static_tail_start_ >= 0 ?
          std::max(0.0, (sample_count_ - 1 - static_tail_start_) *
          sample_period_) : 0.0;
        stream << "KEEP STILL " << held_seconds << " / " << hold_seconds << " s";
        return stream.str();
      }
    case PoseTailState::kAccepted:
      stream << "MOVE NOW: pose " << pose_count << " / " << min_intervals_
             << " captured, rotate >" << static_cast<int>(kMinPoseAngleDegrees)
             << " deg to the next pose";
      return stream.str();
    case PoseTailState::kDuplicate:
      return "INVALID POSE: same orientation; rotate >15 deg and re-place";
    case PoseTailState::kInvalid:
      return "INVALID STATIC: re-place the sensor and keep still";
    case PoseTailState::kNoStatic:
    default:
      break;
  }

  const double no_static_timeout = kNoStaticTimeoutFactor * hold_seconds;
  if (pose_count == 0) {
    if (sample_count_ * sample_period_ < no_static_timeout) {
      stream << "KEEP STILL " << hold_seconds << " s to capture the first pose";
      return stream.str();
    }
    return "NO STATIC POSE: check mounting/vibration, re-place and keep still";
  }
  if (recent_moving_) {
    return "MOVE: rotate >15 deg to a new pose, then keep still";
  }
  if (since_last_pose_seconds_ < no_static_timeout) {
    stream << "KEEP STILL " << hold_seconds << " s to capture this pose";
    return stream.str();
  }
  return "NO STATIC POSE: check mounting/vibration, re-place and keep still";
}

}  // namespace ssct_imu_intrinsic_calib
