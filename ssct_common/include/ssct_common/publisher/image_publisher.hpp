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

#include <memory>
#include <string>

#include "opencv2/opencv.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

namespace ssct_common
{

class ImagePublisher
{
public:
  ImagePublisher(
    rclcpp::Node::SharedPtr node, std::string topic_name, size_t buffer_size, std::string frame_id,
    bool enable_compressed = false);
  ~ImagePublisher() = default;

  void publish(const cv::Mat & img, rclcpp::Time time);
  void publish(const cv::Mat & img, double time);
  void publish(const cv::Mat & img);
  bool has_subscribers();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_publisher_;
  std::string frame_id_;
  std::string encoding_{"rgb8"};
  bool enable_compressed_{false};
  std::string compressed_format_{"jpeg"};
};
}  // namespace ssct_common
