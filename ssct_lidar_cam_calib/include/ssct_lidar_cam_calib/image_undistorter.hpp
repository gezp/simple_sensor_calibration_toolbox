// Copyright 2024 Gezp (https://github.com/gezp).
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

#include <Eigen/Dense>

#include <memory>
#include <string>
#include <vector>

#include "opencv2/opencv.hpp"

#include "ssct_common/calib_param/camera_intrinsic_param.hpp"

namespace ssct_lidar_cam_calib
{

// Resamples images into a pinhole or ftheta target model:
// pinhole_radtan -> pinhole, pinhole_equidistant -> ftheta.
class ImageUndistorter
{
public:
  bool init(const ssct_common::CameraIntrinsicParam & source);
  bool process(const cv::Mat & image, cv::Mat & out_image) const;
  const ssct_common::CameraIntrinsicParam & target_param() const {return target_;}
  std::string error_message() const {return error_message_;}

private:
  bool build_maps();
  bool unproject_target(double u, double v, Eigen::Vector3d & ray) const;
  bool project_source(const Eigen::Vector3d & ray, double & u, double & v) const;

private:
  ssct_common::CameraIntrinsicParam source_;
  ssct_common::CameraIntrinsicParam target_;
  cv::Mat map_x_;
  cv::Mat map_y_;
  std::string error_message_;
};

}  // namespace ssct_lidar_cam_calib
