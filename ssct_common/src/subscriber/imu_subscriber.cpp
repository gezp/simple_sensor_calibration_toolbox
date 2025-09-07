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

#include "ssct_common/subscriber/imu_subscriber.hpp"

namespace ssct_common
{
ImuSubscriber::ImuSubscriber(rclcpp::Node::SharedPtr node, std::string topic_name, size_t buff_size)
: node_(node)
{
  subscriber_ = node_->create_subscription<sensor_msgs::msg::Imu>(
    topic_name, buff_size, std::bind(&ImuSubscriber::msg_callback, this, std::placeholders::_1));
}

void ImuSubscriber::msg_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg_ptr)
{
  ImuData data;
  data.time = rclcpp::Time(imu_msg_ptr->header.stamp).seconds();
  data.angular_velocity[0] = imu_msg_ptr->angular_velocity.x;
  data.angular_velocity[1] = imu_msg_ptr->angular_velocity.y;
  data.angular_velocity[2] = imu_msg_ptr->angular_velocity.z;
  data.linear_acceleration[0] = imu_msg_ptr->linear_acceleration.x;
  data.linear_acceleration[1] = imu_msg_ptr->linear_acceleration.y;
  data.linear_acceleration[2] = imu_msg_ptr->linear_acceleration.z;
  buffer_mutex_.lock();
  buffer_.push_back(data);
  buffer_mutex_.unlock();
}

void ImuSubscriber::read(std::deque<ImuData> & output)
{
  buffer_mutex_.lock();
  if (buffer_.size() > 0) {
    output.insert(output.end(), buffer_.begin(), buffer_.end());
    buffer_.clear();
  }
  buffer_mutex_.unlock();
}

void ImuSubscriber::clear()
{
  buffer_mutex_.lock();
  buffer_.clear();
  buffer_mutex_.unlock();
}

}  // namespace ssct_common
