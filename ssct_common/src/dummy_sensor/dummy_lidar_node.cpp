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

#include "ssct_common/dummy_sensor/dummy_lidar_node.hpp"
#include <filesystem>
#include <algorithm>

namespace ssct_common
{

DummyLidarNode::DummyLidarNode(const rclcpp::NodeOptions & options)
{
  node_ = std::make_shared<rclcpp::Node>("dummy_lidar_node", options);
  node_->declare_parameter("frame_id", frame_id_);
  node_->declare_parameter("data_dir", data_dir_);
  node_->declare_parameter("rate", rate_);
  node_->declare_parameter("timestamp", timestamp_);
  node_->declare_parameter("loop", loop_);

  node_->get_parameter("frame_id", frame_id_);
  node_->get_parameter("data_dir", data_dir_);
  node_->get_parameter("rate", rate_);
  node_->get_parameter("timestamp", timestamp_);
  node_->get_parameter("loop", loop_);
  // read data
  if (data_dir_.empty()) {
    RCLCPP_FATAL(node_->get_logger(), "data_dir is empty.");
    return;
  }
  if (!read_data()) {
    RCLCPP_FATAL(node_->get_logger(), "no pcds.");
    return;
  }
  // publisher
  std::string topic_name = "/sensor/" + frame_id_ + "/pointcloud";
  cloud_pub_ = std::make_shared<CloudPublisher>(node_, topic_name, 100, frame_id_);
  // create timer
  auto period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / rate_));
  auto timer_callback = [this]() {
      if (pointclouds_.empty()) {
        return;
      }
      if (loop_ && current_idx_ == pointclouds_.size()) {
        current_idx_ = 0;
      }
      if (current_idx_ < pointclouds_.size()) {
        double t = timestamp_ + current_idx_ * 1000.0 / rate_;
        cloud_pub_->publish(*pointclouds_[current_idx_], t);
        current_idx_++;
      }
    };
  timer_ = node_->create_wall_timer(period_ms, timer_callback);
}

bool DummyLidarNode::read_data()
{
  if (data_dir_.empty()) {
    return false;
  }
  // get filepath
  std::vector<std::string> filepaths;
  for (const auto & entry : std::filesystem::directory_iterator(data_dir_)) {
    if (entry.is_regular_file() && entry.path().extension() == ".pcd") {
      // std::cout << entry.path() << std::endl; // 输出jpg文件路径
      filepaths.push_back(entry.path().string());
    }
  }
  if (filepaths.empty()) {
    return false;
  }
  std::sort(filepaths.begin(), filepaths.end());
  // read pcd
  pointclouds_.clear();
  for (auto & filepath : filepaths) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::io::loadPCDFile(filepath, *cloud);
    pointclouds_.push_back(cloud);
  }
  return true;
}

}  // namespace ssct_common
