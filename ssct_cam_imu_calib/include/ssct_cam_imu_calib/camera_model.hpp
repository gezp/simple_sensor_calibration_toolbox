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

#include <Eigen/Geometry>

#include <string>
#include <vector>

#include "opencv2/opencv.hpp"
#include "ssct_common/calib_param/camera_intrinsic_param.hpp"

namespace ssct_cam_imu_calib
{

// Supports pinhole, pinhole_radtan and ftheta_kb. Points passed to the
// optimizer are undistorted normalized coordinates; the optimizer therefore
// does not need the distortion model.
class CameraModel
{
public:
  bool init(const ssct_common::CameraIntrinsicParam & param);
  const std::string & error_message() const {return error_message_;}
  const cv::Mat & camera_matrix() const {return camera_matrix_;}
  double focal_x() const {return camera_matrix_.at<double>(0, 0);}
  double focal_y() const {return camera_matrix_.at<double>(1, 1);}
  bool undistort_points(
    const std::vector<cv::Point2f> & image_points,
    std::vector<cv::Point2f> & normalized_points) const;
  bool project_points(
    const std::vector<cv::Point3f> & object_points,
    const Eigen::Matrix4d & T_camera_board,
    std::vector<cv::Point2f> & image_points) const;

private:
  bool fisheye_{false};
  cv::Mat camera_matrix_;
  cv::Mat distortion_;
  std::string error_message_;
};

}  // namespace ssct_cam_imu_calib
