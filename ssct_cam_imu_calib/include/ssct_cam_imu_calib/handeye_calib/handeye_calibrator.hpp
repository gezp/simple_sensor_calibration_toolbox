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

#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "ssct_cam_imu_calib/aruco_detector.hpp"
#include "ssct_cam_imu_calib/cam_imu_calibrator_interface.hpp"
#include "ssct_cam_imu_calib/camera_model.hpp"
#include "ssct_cam_imu_calib/handeye_calib/handeye_optimizer.hpp"
#include "ssct_cam_imu_calib/handeye_calib/imu_integration.hpp"
#include "ssct_cam_imu_calib/handeye_calib/static_imu_collector.hpp"
#include "ssct_common/calib_param/camera_intrinsic_param.hpp"
#include "ssct_common/sensor_data/image_data.hpp"
#include "ssct_common/sensor_data/imu_data.hpp"

namespace ssct_cam_imu_calib
{

class HandEyeCalibrator : public CamImuCalibratorInterface
{
public:
  explicit HandEyeCalibrator(const YAML::Node & config);

  void set_camera_intrinsic_param(const ssct_common::CameraIntrinsicParam & camera);

  bool add_data(
    const std::vector<ssct_common::ImageData> & images,
    const std::vector<ssct_common::ImuData> & imu) override;
  bool ready_to_optimize() const override {return ready_to_optimize_;}
  bool optimize() override;
  uint8_t progress() const override {return progress_;}
  const std::string & message() const override {return message_;}
  ssct_common::ExtrinsicParam get_result() const override {return result_;}
  void reset() override;
  cv::Mat get_new_debug_image() const override {return debug_image_;}

private:
  // One still segment with a board detection: the anchor observation and the
  // integration prior of pairs starting there.
  struct StaticState
  {
    double time{0.0};
    Eigen::Matrix4d T_camera_board{Eigen::Matrix4d::Identity()};
    Eigen::Vector3d gyro_bias{Eigen::Vector3d::Zero()};
    Eigen::Vector3d gravity{Eigen::Vector3d(0.0, 0.0, -9.80665)};
  };

  void process_image(const ssct_common::ImageData & image);
  bool is_valid_pair(const StaticState & state_a, const StaticState & state_b) const;
  // Integrates imu_data_ from start_time to end_time.
  bool integrate_imu(
    double start_time, double end_time,
    const ImuNavState & initial_state, ImuNavState & end_state) const;
  // Turns still segments whose images have all arrived into A->B pairs.
  void process_segments();
  // Updates progress and message, and returns whether optimization can start.
  bool check_ready_to_optimize();

private:
  CameraModel camera_model_;
  std::unique_ptr<ArucoDetector> detector_;
  StaticImuCollector segment_collector_;
  HandEyeOptimizer optimizer_;
  double min_image_interval_{0.05};
  double min_rotation_deg_{5.0};
  double max_pair_interval_{5.0};
  int min_pairs_{6};
  // Keyed by timestamp, capped at kMaxImuSamples; oldest samples are dropped.
  std::map<double, ssct_common::ImuData> imu_data_;
  std::deque<BoardObservation> observations_;
  // Still segments waiting for the images up to their end time.
  std::deque<StaticImuSegment> pending_segments_;
  // Previous valid still segment; pairs integrate from here.
  StaticState previous_state_;
  bool has_previous_state_{false};
  std::vector<HandEyePair> pairs_;
  bool ready_to_optimize_{false};
  size_t static_segment_count_{0};
  double last_image_time_{-1.0};
  double last_processed_image_time_{-1.0};
  bool last_detected_{false};
  double last_reprojection_error_{0.0};
  cv::Mat debug_image_;
  ssct_common::ExtrinsicParam result_;
  uint8_t progress_{0};
  std::string message_;
};

}  // namespace ssct_cam_imu_calib
