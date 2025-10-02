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

#include <thread>
#include <memory>
#include <string>
#include <deque>
#include <vector>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "ssct_common/publisher/image_publisher.hpp"
#include "ssct_common/subscriber/image_subscriber.hpp"
#include "ssct_common/subscriber/cloud_subscriber.hpp"
#include "ssct_common/calib_param_manager.hpp"
#include "ssct_interfaces/msg/calibration_status.hpp"
#include "ssct_interfaces/msg/calibration_command.hpp"
#include "ssct_lidar_cam_manual_calib/lidar_projector.hpp"

namespace ssct_lidar_cam_manual_calib
{

class CalibrationNode
{
  struct SensorData
  {
    double time;
    cv::Mat image;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud;
  };

public:
  explicit CalibrationNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CalibrationNode();
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
  {
    return node_->get_node_base_interface();
  }

private:
  bool get_nearest_image(double time, double & image_timestamp, cv::Mat & image);
  bool read_data();
  void clear_data();
  void process_command(const ssct_interfaces::msg::CalibrationCommand & msg);
  bool update_calib_param_manager(const std::string & key, float value);
  void update_status_msg(uint8_t state, const std::string & info, uint8_t progress);
  void save_result();
  bool run();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<ssct_common::CloudSubscriber<pcl::PointXYZI>> pointcloud_sub_;
  std::shared_ptr<ssct_common::ImageSubscriber> image_sub_;
  rclcpp::Subscription<ssct_interfaces::msg::CalibrationCommand>::SharedPtr cmd_sub_;
  rclcpp::Publisher<ssct_interfaces::msg::CalibrationStatus>::SharedPtr status_pub_;
  std::shared_ptr<ssct_common::ImagePublisher> debug_image_pub_;
  std::shared_ptr<LidarProjector> lidar_projector_;
  // param
  std::string lidar_frame_id_{"lidar"};
  std::string camera_frame_id_{"camera"};
  std::string initial_calibration_file_;
  std::string output_calibration_file_;
  bool enable_compressed_image_{false};
  size_t max_image_buffer_size_{50};
  // calibration flow thread
  std::unique_ptr<std::thread> run_thread_;
  bool exit_{false};
  // sensor data
  std::deque<ssct_common::CloudSubscriber<pcl::PointXYZI>::MsgData> pointcloud_buffer_;
  std::map<double, ssct_common::ImageData> image_buffer_;
  std::deque<SensorData> sensor_data_buffer_;
  // camera intrinsic data
  std::vector<double> camera_intrinsic_;
  // extrinsic data
  Eigen::Matrix4d T_camera_lidar_;
  // calibration state
  uint8_t state_;
  bool need_optimize_once_;
  ssct_interfaces::msg::CalibrationStatus status_msg_;
  std::mutex status_mutex_;
  // calibration commnad
  std::deque<ssct_interfaces::msg::CalibrationCommand> command_msgs_;
  std::mutex command_mutex_;
};
}  // namespace ssct_lidar_cam_manual_calib
