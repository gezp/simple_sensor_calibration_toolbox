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

#include "ssct_common/publisher/image_publisher.hpp"

#include "cv_bridge/cv_bridge.h"

namespace ssct_common
{

ImagePublisher::ImagePublisher(
  rclcpp::Node::SharedPtr node, std::string topic_name, size_t buffer_size, std::string frame_id,
  bool enable_compressed)
{
  node_ = node;
  frame_id_ = frame_id;
  enable_compressed_ = enable_compressed;
  publisher_ = node_->create_publisher<sensor_msgs::msg::Image>(topic_name, buffer_size);
  if (enable_compressed_) {
    compressed_publisher_ = node_->create_publisher<sensor_msgs::msg::CompressedImage>(
      std::string(publisher_->get_topic_name()) + "/compressed", buffer_size);
  }
}

void ImagePublisher::publish(const cv::Mat & img, rclcpp::Time time)
{
  if (publisher_->get_subscription_count() > 0) {
    sensor_msgs::msg::Image msg;
    cv_bridge::CvImage(std_msgs::msg::Header(), encoding_, img).toImageMsg(msg);
    msg.header.frame_id = frame_id_;
    msg.header.stamp = time;
    publisher_->publish(msg);
  }
  if (enable_compressed_ && compressed_publisher_->get_subscription_count() > 0) {
    sensor_msgs::msg::CompressedImage msg;
    cv_bridge::CvImage(std_msgs::msg::Header(), encoding_, img).toCompressedImageMsg(msg);
    msg.header.frame_id = frame_id_;
    msg.header.stamp = time;
    msg.format = compressed_format_;
    compressed_publisher_->publish(msg);
  }
}

void ImagePublisher::publish(const cv::Mat & img, double time)
{
  rclcpp::Time ros_time(static_cast<uint64_t>(time * 1e9));
  publish(img, ros_time);
}

void ImagePublisher::publish(const cv::Mat & img)
{
  publish(img, node_->get_clock()->now());
}

bool ImagePublisher::has_subscribers()
{
  return publisher_->get_subscription_count() > 0;
}

}  // namespace ssct_common
