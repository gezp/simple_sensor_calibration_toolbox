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
#include <memory>
#include <string>
#include <vector>

#include "ssct_cam_imu_calib/aruco_detector.hpp"
#include "ssct_cam_imu_calib/cam_imu_calibrator_interface.hpp"
#include "ssct_cam_imu_calib/camera_model.hpp"
#include "ssct_cam_imu_calib/kalibr_calib/kalibr_optimizer.hpp"
#include "ssct_common/calib_param/camera_intrinsic_param.hpp"
#include "ssct_common/calib_param/extrinsic_param.hpp"
#include "ssct_common/calib_param/imu_intrinsic_param.hpp"
#include "ssct_common/sensor_data/image_data.hpp"
#include "ssct_common/sensor_data/imu_data.hpp"

namespace ssct_cam_imu_calib
{

class KalibrCalibrator : public CamImuCalibratorInterface
{
public:
  explicit KalibrCalibrator(const YAML::Node & config);

  void set_camera_intrinsic_param(const ssct_common::CameraIntrinsicParam & camera);
  void set_imu_intrinsic_param(const ssct_common::ImuIntrinsicParam & imu);

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
  void process_image(const ssct_common::ImageData & image);
  void select_observation(const BoardObservation & observation);
  bool check_ready_to_optimize();

private:
  YAML::Node config_node_;
  CameraModel camera_model_;
  std::unique_ptr<ArucoDetector> detector_;
  KalibrOptimizer optimizer_;
  int min_keyframes_{20};
  double min_rotation_deg_{10.0};
  double min_image_interval_{0.05};
  double min_collection_duration_{60.0};
  std::vector<ssct_common::ImuData> imu_data_;
  std::vector<BoardObservation> selected_observations_;
  double rotation_excitation_{0.0};
  bool ready_to_optimize_{false};
  double first_detection_time_{-1.0};
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
