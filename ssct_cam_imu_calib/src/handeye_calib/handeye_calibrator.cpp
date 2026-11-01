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

#include "ssct_cam_imu_calib/handeye_calib/handeye_calibrator.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <sstream>
#include <stdexcept>

#include "ssct_cam_imu_calib/handeye_calib/imu_integration.hpp"
#include "ssct_cam_imu_calib/math_utils.hpp"

namespace ssct_cam_imu_calib
{
namespace
{

constexpr size_t kMaxImuSamples = 100000;
constexpr size_t kMaxPendingSegments = 5;

}  // namespace

HandEyeCalibrator::HandEyeCalibrator(const YAML::Node & config)
{
  detector_ = std::make_unique<ArucoDetector>(config["aruco_detector"]);
  optimizer_ = HandEyeOptimizer(config["optimizer"]);
  min_image_interval_ = config["min_image_interval"].as<double>();
  max_pair_interval_ = config["max_pair_interval"].as<double>();
  min_rotation_deg_ = config["min_rotation_deg"].as<double>();
  min_pairs_ = config["min_pairs"].as<int>();
  segment_collector_ = StaticImuCollector(config["static_imu_collector"]);
  reset();
}

void HandEyeCalibrator::set_camera_intrinsic_param(
  const ssct_common::CameraIntrinsicParam & camera)
{
  if (!camera_model_.init(camera)) {
    throw std::runtime_error("failed to init camera model: " + camera_model_.error_message());
  }
  detector_->set_camera_model(camera_model_);
}

bool HandEyeCalibrator::add_data(
  const std::vector<ssct_common::ImageData> & images,
  const std::vector<ssct_common::ImuData> & imu)
{
  for (const auto & sample : imu) {
    imu_data_.emplace(sample.time, sample);
  }
  while (imu_data_.size() > kMaxImuSamples) {
    imu_data_.erase(imu_data_.begin());
  }
  if (segment_collector_.collect(imu)) {
    pending_segments_.push_back(segment_collector_.segment());
    if (pending_segments_.size() > kMaxPendingSegments) {
      pending_segments_.pop_front();
    }
    segment_collector_.reset();
  }
  for (const auto & image : images) {
    last_image_time_ = image.time;
    if (last_processed_image_time_ >= 0.0 &&
      image.time - last_processed_image_time_ < min_image_interval_)
    {
      continue;
    }
    last_processed_image_time_ = image.time;
    process_image(image);
  }
  process_segments();
  ready_to_optimize_ = check_ready_to_optimize();
  return true;
}

bool HandEyeCalibrator::optimize()
{
  optimizer_.add_data(pairs_);
  const bool success = optimizer_.optimize();
  progress_ = 100;
  message_ = optimizer_.message();
  if (success) {
    result_ = optimizer_.get_result();
  }
  return success;
}

void HandEyeCalibrator::reset()
{
  imu_data_.clear();
  observations_.clear();
  last_image_time_ = -1.0;
  last_processed_image_time_ = -1.0;
  last_detected_ = false;
  last_reprojection_error_ = 0.0;
  pending_segments_.clear();
  previous_state_ = StaticState();
  has_previous_state_ = false;
  pairs_.clear();
  ready_to_optimize_ = false;
  static_segment_count_ = 0;
  segment_collector_.reset();
  debug_image_.release();
  result_ = ssct_common::ExtrinsicParam();
  progress_ = 0;
  message_ = "waiting for images.";
}

void HandEyeCalibrator::process_image(const ssct_common::ImageData & image)
{
  BoardObservation observation;
  observation.time = image.time;
  last_detected_ = detector_->detect(image.image, observation);
  if (last_detected_) {
    observations_.push_back(observation);
    last_reprojection_error_ = observation.reprojection_error;
  }

  debug_image_ = detector_->draw(image.image, observation);
}

bool HandEyeCalibrator::is_valid_pair(
  const StaticState & state_a, const StaticState & state_b) const
{
  const double interval = state_b.time - state_a.time;
  if (interval <= 0.0 || interval > max_pair_interval_) {
    return false;
  }
  return rotation_angle(
    state_a.T_camera_board.block<3, 3>(0, 0).transpose() *
    state_b.T_camera_board.block<3, 3>(0, 0)) >=
         degrees_to_radians(min_rotation_deg_);
}

bool HandEyeCalibrator::integrate_imu(
  double start_time, double end_time,
  const ImuNavState & initial_state, ImuNavState & end_state) const
{
  auto first = imu_data_.lower_bound(start_time);
  if (first == imu_data_.end()) {
    return false;
  }
  if (first != imu_data_.begin()) {
    --first;
  }
  const auto last = imu_data_.upper_bound(end_time);
  if (first == last || std::next(first) == last) {
    return false;
  }

  ImuIntegrator integrator;
  integrator.reset(initial_state, first->second);
  for (auto it = std::next(first); it != last; ++it) {
    integrator.integrate(it->second);
  }
  end_state = integrator.state();
  return true;
}

void HandEyeCalibrator::process_segments()
{
  while (!pending_segments_.empty() &&
    last_image_time_ >= pending_segments_.front().end)
  {
    // Without any board detection there is nothing to anchor a pair.
    if (observations_.empty()) {
      return;
    }
    const StaticImuSegment segment = pending_segments_.front();
    pending_segments_.pop_front();
    static_segment_count_++;
    const double middle = 0.5 * (segment.start + segment.end);
    const BoardObservation * observation = nullptr;
    double best_distance = std::numeric_limits<double>::infinity();
    for (const auto & candidate : observations_) {
      if (candidate.time < segment.start || candidate.time > segment.end) {
        continue;
      }
      const double distance = std::abs(candidate.time - middle);
      if (distance < best_distance) {
        best_distance = distance;
        observation = &candidate;
      }
    }
    if (observation != nullptr) {
      const StaticState current{
        observation->time, observation->T_camera_board,
        segment.gyro_bias, segment.gravity};
      if (has_previous_state_ && is_valid_pair(previous_state_, current)) {
        ImuNavState initial_state;
        initial_state.gyro_bias = previous_state_.gyro_bias;
        initial_state.gravity = previous_state_.gravity;
        ImuNavState end_state;
        if (integrate_imu(
            previous_state_.time, current.time, initial_state, end_state) &&
          end_state.orientation.allFinite() && end_state.position.allFinite())
        {
          HandEyePair pair;
          pair.T_cam_a_b = previous_state_.T_camera_board *
            current.T_camera_board.inverse();
          pair.T_imu_a_b.block<3, 3>(0, 0) = end_state.orientation;
          pair.T_imu_a_b.block<3, 1>(0, 3) = end_state.position;
          pair.position_drift = end_state.position.norm();
          pairs_.push_back(pair);
        }
      }
      previous_state_ = current;
      has_previous_state_ = true;
    }
    // Observations up to this segment are no longer needed.
    while (!observations_.empty() && observations_.front().time <= segment.end) {
      observations_.pop_front();
    }
  }
}

bool HandEyeCalibrator::check_ready_to_optimize()
{
  progress_ = min_pairs_ > 0 ?
    static_cast<uint8_t>(std::min(95.0, 95.0 * pairs_.size() / min_pairs_)) : 0;
  const std::string detection = last_detected_ ?
    ", last reprojection " + std::to_string(last_reprojection_error_) + " px" :
    ", target not detected";

  if (last_image_time_ < 0.0) {
    message_ = "waiting for images.";
    return false;
  }
  if (imu_data_.empty()) {
    message_ = "waiting for IMU data." + detection;
    return false;
  }
  if (pairs_.size() < static_cast<size_t>(min_pairs_)) {
    std::ostringstream stream;
    stream << "collecting: static segments " << static_segment_count_
           << ", pending " << pending_segments_.size()
           << ", pairs " << pairs_.size() << " / " << min_pairs_;
    if (segment_collector_.ready()) {
      stream << ", still segment ready, move to next position";
    }
    message_ = stream.str() + detection;
    return false;
  }
  progress_ = 95;
  message_ = "ready to optimize." + detection;
  return true;
}

}  // namespace ssct_cam_imu_calib
