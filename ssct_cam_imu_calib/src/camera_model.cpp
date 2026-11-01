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

#include "ssct_cam_imu_calib/camera_model.hpp"

#include <cmath>

namespace ssct_cam_imu_calib
{

bool CameraModel::init(const ssct_common::CameraIntrinsicParam & param)
{
  if (param.intrinsics.size() < 4) {
    error_message_ = "camera intrinsics must contain fx, fy, cx, cy";
    return false;
  }
  if (!std::isfinite(param.intrinsics[0]) || !std::isfinite(param.intrinsics[1]) ||
    param.intrinsics[0] <= 0.0 || param.intrinsics[1] <= 0.0)
  {
    error_message_ = "camera focal length must be positive and finite";
    return false;
  }
  if (param.type == "pinhole" || param.type == "pinhole_radtan") {
    fisheye_ = false;
    camera_matrix_ = (cv::Mat_<double>(3, 3) <<
      param.intrinsics[0], 0.0, param.intrinsics[2],
      0.0, param.intrinsics[1], param.intrinsics[3],
      0.0, 0.0, 1.0);
    if (param.type == "pinhole") {
      distortion_ = cv::Mat::zeros(1, 5, CV_64F);
    } else {
      distortion_ = cv::Mat::zeros(1, 5, CV_64F);
      for (size_t i = 0; i < param.distortion_coeffs.size() && i < 5; i++) {
        distortion_.at<double>(0, static_cast<int>(i)) = param.distortion_coeffs[i];
      }
    }
  } else if (param.type == "ftheta_kb") {
    fisheye_ = true;
    camera_matrix_ = (cv::Mat_<double>(3, 3) <<
      param.intrinsics[0], 0.0, param.intrinsics[2],
      0.0, param.intrinsics[1], param.intrinsics[3],
      0.0, 0.0, 1.0);
    distortion_ = cv::Mat::zeros(1, 4, CV_64F);
    for (size_t i = 0; i < param.distortion_coeffs.size() && i < 4; i++) {
      distortion_.at<double>(0, static_cast<int>(i)) = param.distortion_coeffs[i];
    }
  } else {
    error_message_ = "unsupported camera model: " + param.type;
    return false;
  }
  return true;
}

bool CameraModel::undistort_points(
  const std::vector<cv::Point2f> & image_points,
  std::vector<cv::Point2f> & normalized_points) const
{
  if (image_points.empty()) {
    return false;
  }
  try {
    if (fisheye_) {
      cv::fisheye::undistortPoints(image_points, normalized_points, camera_matrix_, distortion_);
    } else {
      cv::undistortPoints(image_points, normalized_points, camera_matrix_, distortion_);
    }
  } catch (const cv::Exception &) {
    return false;
  }
  return normalized_points.size() == image_points.size();
}

bool CameraModel::project_points(
  const std::vector<cv::Point3f> & object_points,
  const Eigen::Matrix4d & T_camera_board,
  std::vector<cv::Point2f> & image_points) const
{
  if (object_points.empty()) {
    return false;
  }
  if (!T_camera_board.allFinite()) {
    return false;
  }
  cv::Mat rotation_vector;
  cv::Mat translation_vector;
  translation_vector = (cv::Mat_<double>(3, 1) <<
    T_camera_board(0, 3), T_camera_board(1, 3), T_camera_board(2, 3));
  try {
    cv::Rodrigues(
      cv::Matx33d(
        T_camera_board(0, 0), T_camera_board(0, 1), T_camera_board(0, 2),
        T_camera_board(1, 0), T_camera_board(1, 1), T_camera_board(1, 2),
        T_camera_board(2, 0), T_camera_board(2, 1), T_camera_board(2, 2)),
      rotation_vector);
    if (fisheye_) {
      cv::fisheye::projectPoints(
        object_points, image_points, rotation_vector, translation_vector,
        camera_matrix_, distortion_);
    } else {
      cv::projectPoints(
        object_points, rotation_vector, translation_vector,
        camera_matrix_, distortion_, image_points);
    }
  } catch (const cv::Exception &) {
    return false;
  }
  return image_points.size() == object_points.size();
}

}  // namespace ssct_cam_imu_calib
