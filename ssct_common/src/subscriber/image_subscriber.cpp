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

#include "ssct_common/subscriber/image_subscriber.hpp"

#include "cv_bridge/cv_bridge.h"

namespace ssct_common
{

ImageSubscriber::ImageSubscriber(
  rclcpp::Node::SharedPtr node, std::string topic_name, size_t buffer_size, bool enable_compressed)
{
  node_ = node;
  buffer_size_ = buffer_size;
  enable_compressed_ = enable_compressed;
  if (enable_compressed) {
    auto msg_callback = [this](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        ImageData data;
        data.time = rclcpp::Time(msg->header.stamp).seconds();
        data.image = cv_bridge::toCvCopy(*msg)->image;
        buffer_mutex_.lock();
        buffer_.push_back(data);
        if (buffer_.size() > buffer_size_) {
          buffer_.pop_front();
        }
        buffer_mutex_.unlock();
      };
    compressed_subscriber_ = node_->create_subscription<sensor_msgs::msg::CompressedImage>(
      topic_name, buffer_size, msg_callback);
  } else {
    auto msg_callback = [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        ImageData data;
        data.time = rclcpp::Time(msg->header.stamp).seconds();
        data.image = cv_bridge::toCvCopy(*msg)->image;
        buffer_mutex_.lock();
        buffer_.push_back(data);
        if (buffer_.size() > buffer_size_) {
          buffer_.pop_front();
        }
        buffer_mutex_.unlock();
      };
    subscriber_ =
      node_->create_subscription<sensor_msgs::msg::Image>(topic_name, buffer_size, msg_callback);
  }
}

const char * ImageSubscriber::get_topic_name()
{
  if (enable_compressed_) {
    return compressed_subscriber_->get_topic_name();
  } else {
    return subscriber_->get_topic_name();
  }
}

void ImageSubscriber::read(std::deque<ImageData> & output)
{
  buffer_mutex_.lock();
  if (buffer_.size() > 0) {
    output.insert(output.end(), buffer_.begin(), buffer_.end());
    buffer_.clear();
  }
  buffer_mutex_.unlock();
}

void ImageSubscriber::read(std::map<double, ImageData> & output)
{
  buffer_mutex_.lock();
  if (buffer_.size() > 0) {
    for (auto & msg : buffer_) {
      output.insert({msg.time, msg});
    }
    buffer_.clear();
  }
  buffer_mutex_.unlock();
}

void ImageSubscriber::clear()
{
  buffer_mutex_.lock();
  buffer_.clear();
  buffer_mutex_.unlock();
}

}  // namespace ssct_common
