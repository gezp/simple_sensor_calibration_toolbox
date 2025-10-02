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

#include <deque>
#include <mutex>
#include <string>
#include <map>

#include "opencv2/opencv.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

#include "ssct_common/sensor_data/image_data.hpp"

namespace ssct_common
{

class ImageSubscriber
{
public:
  ImageSubscriber(
    rclcpp::Node::SharedPtr node, std::string topic_name, size_t buffer_size,
    bool enable_compressed = false);

  const char * get_topic_name();
  void read(std::deque<ImageData> & output);
  void read(std::map<double, ImageData> & output);
  void clear();

private:
  rclcpp::Node::SharedPtr node_;
  bool enable_compressed_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_subscriber_;
  std::deque<ImageData> buffer_;
  size_t buffer_size_;
  std::mutex buffer_mutex_;
};
}  // namespace ssct_common
