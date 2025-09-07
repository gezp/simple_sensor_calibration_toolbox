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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <deque>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace ssct_common
{

template<typename PointT>
class CloudSubscriber
{
public:
  struct MsgData
  {
    double time;
    typename pcl::PointCloud<PointT>::Ptr pointcloud;
  };
  CloudSubscriber(rclcpp::Node::SharedPtr node, std::string topic_name, size_t buffer_size)
  {
    node_ = node;
    buffer_size_ = buffer_size;
    auto msg_callback = [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        MsgData data;
        data.time = rclcpp::Time(msg->header.stamp).seconds();
        data.pointcloud.reset(new pcl::PointCloud<PointT>());
        pcl::fromROSMsg(*msg, *(data.pointcloud));
        buffer_mutex_.lock();
        buffer_.push_back(data);
        if (buffer_.size() > buffer_size_) {
          buffer_.pop_front();
        }
        buffer_mutex_.unlock();
      };
    subscriber_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic_name, buffer_size, msg_callback);
  }

  const char * get_topic_name() {return subscriber_->get_topic_name();}

  void read(std::deque<MsgData> & output)
  {
    buffer_mutex_.lock();
    if (buffer_.size() > 0) {
      output.insert(output.end(), buffer_.begin(), buffer_.end());
      buffer_.clear();
    }
    buffer_mutex_.unlock();
  }

  void clear()
  {
    buffer_mutex_.lock();
    buffer_.clear();
    buffer_mutex_.unlock();
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
  std::deque<MsgData> buffer_;
  size_t buffer_size_;
  std::mutex buffer_mutex_;
};
}  // namespace ssct_common
