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

#include "ssct_common/dummy_sensor/dummy_camera_node.hpp"

#include <filesystem>
#include <set>

namespace ssct_common
{

DummyCameraNode::DummyCameraNode(const rclcpp::NodeOptions & options)
{
  node_ = std::make_shared<rclcpp::Node>("dummy_camera_node", options);
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
    RCLCPP_FATAL(node_->get_logger(), "no images.");
    return;
  }
  // publisher
  std::string topic_name = "/sensor/" + frame_id_ + "/image";
  image_pub_ = std::make_shared<ImagePublisher>(node_, topic_name, 100, frame_id_, true);
  // create timer
  auto period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / rate_));
  auto timer_callback = [this]() {
      if (loop_ && current_idx_ == images_.size()) {
        current_idx_ = 0;
      }
      if (current_idx_ < images_.size()) {
        double t = timestamp_ + current_idx_ * 1000.0 / rate_;
        image_pub_->publish(images_[current_idx_], t);
        current_idx_++;
      }
    };
  timer_ = node_->create_wall_timer(period_ms, timer_callback);
}

bool DummyCameraNode::read_data()
{
  // read image one by one
  std::map<int, cv::Mat> images;
  for (const auto & entry : std::filesystem::directory_iterator(data_dir_)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    // read image
    auto img = cv::imread(entry.path().string());
    if (img.data == NULL) {
      continue;
    }
    try {
      int num = std::stoi(entry.path().stem().string());
      images[num] = img;
    } catch (...) {
      images[-1] = img;
    }
  }

  // copy to vector
  images_.clear();
  for (auto & [k, v] : images) {
    images_.push_back(v);
  }
  return !images_.empty();
}

}  // namespace ssct_common
