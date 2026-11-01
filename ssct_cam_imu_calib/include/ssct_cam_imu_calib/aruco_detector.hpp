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

#include <Eigen/Geometry>

#include <string>
#include <vector>

#include "opencv2/aruco.hpp"
#include "opencv2/aruco/charuco.hpp"
#include "opencv2/opencv.hpp"

#include "ssct_cam_imu_calib/camera_model.hpp"

namespace ssct_cam_imu_calib
{

struct BoardObservation
{
  double time{0.0};
  std::vector<cv::Point2f> image_points;
  std::vector<cv::Point2f> normalized_points;
  std::vector<cv::Point3f> object_points;
  Eigen::Matrix4d T_camera_board{Eigen::Matrix4d::Identity()};
  double reprojection_error{0.0};
};

class ArucoDetector
{
public:
  explicit ArucoDetector(const YAML::Node & config);
  void set_camera_model(const CameraModel & camera_model);
  bool detect(const cv::Mat & image, BoardObservation & observation) const;
  cv::Mat draw(const cv::Mat & image, const BoardObservation & observation) const;
  const CameraModel & camera_model() const {return *camera_model_;}
  int min_corners() const {return min_corners_;}

private:
  enum class TargetType { CHARUCO, GRIDBOARD };

  bool detect_charuco(
    const cv::Mat & image, const std::vector<std::vector<cv::Point2f>> & marker_corners,
    const std::vector<int> & marker_ids, BoardObservation & observation) const;
  bool detect_gridboard(
    const std::vector<std::vector<cv::Point2f>> & marker_corners,
    const std::vector<int> & marker_ids, BoardObservation & observation) const;
  bool estimate_pose(BoardObservation & observation) const;

  TargetType target_type_{TargetType::CHARUCO};
  int min_corners_{12};
  double max_reprojection_error_{2.0};
  const CameraModel * camera_model_{nullptr};
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::CharucoBoard> charuco_board_;
  cv::Ptr<cv::aruco::GridBoard> grid_board_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
};

}  // namespace ssct_cam_imu_calib
