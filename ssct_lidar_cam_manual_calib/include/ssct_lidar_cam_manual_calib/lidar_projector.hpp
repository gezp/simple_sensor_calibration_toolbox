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

#include <yaml-cpp/yaml.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

#include <memory>
#include <string>
#include <vector>

#include "opencv2/opencv.hpp"

namespace ssct_lidar_cam_manual_calib
{

class LidarProjector
{
public:
  explicit LidarProjector(const YAML::Node & config);
  bool project(
    const pcl::PointCloud<pcl::PointXYZI> & pointcloud,
    const std::vector<double> & camera_intrinsic, const Eigen::Matrix4d & T_lidar_camera,
    cv::Mat & out_image);
  std::string error_message() {return error_message_;}

private:
  cv::Scalar generate_color(float value);

private:
  bool use_intensity_color_{false};
  int point_size_{3};
  std::string error_message_;
};

}  // namespace ssct_lidar_cam_manual_calib
