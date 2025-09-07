// Copyright 2025 Gezp (https://github.com/gezp).
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

#include "ssct_common/dummy_sensor/dummy_imu_node.hpp"

#include <fstream>
#include <sstream>

namespace ssct_common
{

DummyImuNode::DummyImuNode(const rclcpp::NodeOptions & options)
{
  node_ = std::make_shared<rclcpp::Node>("dummy_imu_node", options);
  node_->declare_parameter("frame_id", frame_id_);
  node_->declare_parameter("data_file", data_file_);
  node_->declare_parameter("rate", rate_);
  node_->declare_parameter("timestamp", timestamp_);
  node_->declare_parameter("loop", loop_);

  node_->get_parameter("frame_id", frame_id_);
  node_->get_parameter("data_file", data_file_);
  node_->get_parameter("rate", rate_);
  node_->get_parameter("timestamp", timestamp_);
  node_->get_parameter("loop", loop_);
  // read data
  if (data_file_.empty()) {
    RCLCPP_FATAL(node_->get_logger(), "data_file is empty.");
    return;
  }
  if (!read_data()) {
    RCLCPP_FATAL(node_->get_logger(), "no imus.");
    return;
  }
  // publisher
  std::string topic_name = "/sensor/" + frame_id_ + "/imu";
  imu_pub_ = std::make_shared<ImuPublisher>(node_, topic_name, 100, frame_id_);
  // create timer
  auto period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / rate_));
  auto timer_callback = [this]() {
      if (loop_ && current_idx_ == imus_.size()) {
        current_idx_ = 0;
      }
      if (current_idx_ < imus_.size()) {
        imu_pub_->publish(imus_[current_idx_]);
        current_idx_++;
      }
    };
  timer_ = node_->create_wall_timer(period_ms, timer_callback);
}

bool DummyImuNode::read_data()
{
  std::ifstream ifs(data_file_);
  if (!ifs.is_open()) {
    RCLCPP_FATAL(node_->get_logger(), "failed to open file %s.", data_file_.c_str());
    return false;
  }
  std::string line;
  while (std::getline(ifs, line)) {
    if (line.empty() || !std::isdigit(line[0])) {
      // skip head line and empty line
      continue;
    }
    std::istringstream ss(line);
    std::string token;
    ImuData imu;
    try {
      std::getline(ss, token, ',');
      imu.time = timestamp_ + std::stod(token);
      std::getline(ss, token, ',');
      imu.angular_velocity.x() = std::stod(token);
      std::getline(ss, token, ',');
      imu.angular_velocity.y() = std::stod(token);
      std::getline(ss, token, ',');
      imu.angular_velocity.z() = std::stod(token);
      std::getline(ss, token, ',');
      imu.linear_acceleration.x() = std::stod(token);
      std::getline(ss, token, ',');
      imu.linear_acceleration.y() = std::stod(token);
      std::getline(ss, token, ',');
      imu.linear_acceleration.z() = std::stod(token);
      imus_.push_back(imu);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(
        node_->get_logger(), "failed parse line: [%s], error: %s", line.c_str(), e.what());
      return false;
    }
  }
  ifs.close();
  return !imus_.empty();
}

}  // namespace ssct_common
