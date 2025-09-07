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

#include "ssct_lidar_cam_manual_calib/lidar_projector.hpp"

namespace ssct_lidar_cam_manual_calib
{

LidarProjector::LidarProjector(const YAML::Node & config)
{
  use_intensity_color_ = config["use_intensity_color"].as<bool>();
  point_size_ = config["point_size"].as<int>();
}

bool LidarProjector::project(
  const pcl::PointCloud<pcl::PointXYZI> & pointcloud, const std::vector<double> & camera_intrinsic,
  const Eigen::Matrix4d & T_camera_lidar, cv::Mat & out_image)
{
  // porject 3d to 2d
  cv::Mat index_img(out_image.rows, out_image.cols, CV_32SC1, -1);
  Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
  K(0, 0) = camera_intrinsic[0];
  K(1, 1) = camera_intrinsic[1];
  K(0, 2) = camera_intrinsic[2];
  K(1, 2) = camera_intrinsic[3];
  std::vector<float> distances(pointcloud.points.size(), 0);
  float max_distance = 0;
  float max_intensity = 0;
  int n = static_cast<int>(pointcloud.points.size());
  for (int i = 0; i < n; i++) {
    auto & p = pointcloud.points[i];
    auto p3d = (T_camera_lidar.block<3, 3>(0, 0) * Eigen::Vector3d(p.x, p.y, p.z) +
      T_camera_lidar.block<3, 1>(0, 3))
      .eval();
    if (p3d.z() < 0) {
      continue;
    }
    distances[i] = p3d.norm();
    auto p2d = (K * p3d / p3d.z()).eval();
    int u = cvRound(p2d.x());
    int v = cvRound(p2d.y());
    if (v >= 0 && v < index_img.rows && u >= 0 && u < index_img.cols) {
      auto & idx = index_img.at<int>(v, u);
      if (idx != -1 && distances[i] > distances[idx]) {
        continue;
      }
      index_img.at<int>(v, u) = i;
      max_distance = std::max(max_distance, distances[i]);
      max_intensity = std::max(max_intensity, p.intensity);
    }
  }
  // draw
  for (int i = 0; i < index_img.rows; i++) {
    for (int j = 0; j < index_img.cols; j++) {
      auto & idx = index_img.at<int>(i, j);
      if (idx != -1) {
        cv::Scalar color;
        if (use_intensity_color_) {
          color = generate_color(pointcloud.points[idx].intensity / max_intensity);
        } else {
          color = generate_color(distances[idx] / max_distance);
        }
        circle(out_image, cv::Point(j, i), point_size_, color, -1);
      }
    }
  }
  return true;
}

cv::Scalar LidarProjector::generate_color(float value)
{
  float posSlope = 255 / 60.0;
  float negSlope = -255 / 60.0;
  value *= 255;
  cv::Vec3f color;
  if (value < 60) {
    color[0] = 255;
    color[1] = posSlope * value + 0;
    color[2] = 0;
  } else if (value < 120) {
    color[0] = negSlope * value + 2 * 255;
    color[1] = 255;
    color[2] = 0;
  } else if (value < 180) {
    color[0] = 0;
    color[1] = 255;
    color[2] = posSlope * value - 2 * 255;
  } else if (value < 240) {
    color[0] = 0;
    color[1] = negSlope * value + 4 * 255;
    color[2] = 255;
  } else if (value < 300) {
    color[0] = posSlope * value - 4 * 255;
    color[1] = 0;
    color[2] = 255;
  } else {
    color[0] = 255;
    color[1] = 0;
    color[2] = negSlope * value + 6 * 255;
  }
  return cv::Scalar(color[0], color[1], color[2]);
}

}  // namespace ssct_lidar_cam_manual_calib
