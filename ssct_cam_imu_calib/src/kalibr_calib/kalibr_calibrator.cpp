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

#include "ssct_cam_imu_calib/kalibr_calib/kalibr_calibrator.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "ssct_cam_imu_calib/math_utils.hpp"

namespace ssct_cam_imu_calib
{

KalibrCalibrator::KalibrCalibrator(const YAML::Node & config)
{
  config_node_ = config;
  detector_ = std::make_unique<ArucoDetector>(config["aruco_detector"]);
  optimizer_ = KalibrOptimizer(config["optimizer"]);
  min_keyframes_ = config["min_keyframes"].as<int>();
  min_rotation_deg_ = config["min_rotation_deg"].as<double>();
  min_image_interval_ = config["min_image_interval"].as<double>();
  min_collection_duration_ = config["min_collection_duration"].as<double>();
  reset();
}

void KalibrCalibrator::set_camera_intrinsic_param(
  const ssct_common::CameraIntrinsicParam & camera)
{
  if (!camera_model_.init(camera)) {
    throw std::runtime_error("failed to init camera model: " + camera_model_.error_message());
  }
  detector_->set_camera_model(camera_model_);
  optimizer_.set_camera_model(camera_model_);
}

void KalibrCalibrator::set_imu_intrinsic_param(
  const ssct_common::ImuIntrinsicParam & imu)
{
  optimizer_.set_imu_intrinsic_param(imu);
}

bool KalibrCalibrator::add_data(
  const std::vector<ssct_common::ImageData> & images,
  const std::vector<ssct_common::ImuData> & imu)
{
  imu_data_.insert(imu_data_.end(), imu.begin(), imu.end());
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
  ready_to_optimize_ = check_ready_to_optimize();
  return true;
}

bool KalibrCalibrator::check_ready_to_optimize()
{
  const double duration = first_detection_time_ >= 0.0 ?
    std::max(0.0, last_image_time_ - first_detection_time_) : 0.0;
  progress_ = min_collection_duration_ > 0.0 ?
    static_cast<uint8_t>(std::min(95.0, 95.0 * duration / min_collection_duration_)) : 0;

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
  if (duration < min_collection_duration_) {
    std::ostringstream stream;
    stream << "collecting: keyframes " << selected_observations_.size()
           << ", duration " << duration << " / " << min_collection_duration_ << " s"
           << detection;
    message_ = stream.str();
    return false;
  }
  const size_t min_frames = static_cast<size_t>(std::max(4, min_keyframes_));
  if (selected_observations_.size() < min_frames ||
    rotation_excitation_ < degrees_to_radians(min_rotation_deg_))
  {
    progress_ = 95;
    message_ = "waiting for enough keyframes and rotation excitation." + detection;
    return false;
  }
  progress_ = 95;
  message_ = "ready to optimize." + detection;
  return true;
}

bool KalibrCalibrator::optimize()
{
  optimizer_.add_data(selected_observations_, imu_data_);
  const bool success = optimizer_.optimize();
  progress_ = 100;
  message_ = "rotation excitation " +
    std::to_string(radians_to_degrees(rotation_excitation_)) + " deg; " +
    optimizer_.message();
  if (success) {
    result_ = optimizer_.get_result();
  }
  return success;
}

void KalibrCalibrator::select_observation(
  const BoardObservation & observation)
{
  if (observation.object_points.size() < 4 ||
    observation.normalized_points.size() != observation.object_points.size())
  {
    return;
  }
  if (!selected_observations_.empty()) {
    const Eigen::Matrix3d reference =
      selected_observations_.front().T_camera_board.block<3, 3>(0, 0);
    const Eigen::Matrix3d rotation = observation.T_camera_board.block<3, 3>(0, 0);
    rotation_excitation_ = std::max(
      rotation_excitation_, rotation_angle(reference.transpose() * rotation));
  }
  selected_observations_.push_back(observation);
}

void KalibrCalibrator::reset()
{
  imu_data_.clear();
  selected_observations_.clear();
  rotation_excitation_ = 0.0;
  ready_to_optimize_ = false;
  first_detection_time_ = -1.0;
  last_image_time_ = -1.0;
  last_processed_image_time_ = -1.0;
  last_detected_ = false;
  last_reprojection_error_ = 0.0;
  debug_image_.release();
  result_ = ssct_common::ExtrinsicParam();
  progress_ = 0;
  message_.clear();
}

void KalibrCalibrator::process_image(const ssct_common::ImageData & image)
{
  BoardObservation observation;
  observation.time = image.time;
  last_detected_ = detector_->detect(image.image, observation);
  if (last_detected_) {
    if (first_detection_time_ < 0.0) {
      first_detection_time_ = image.time;
    }
    select_observation(observation);
    last_reprojection_error_ = observation.reprojection_error;
  }

  debug_image_ = detector_->draw(image.image, observation);
}


}  // namespace ssct_cam_imu_calib
