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

#include <chrono>
#include <thread>
#include <memory>
#include <string>
#include <deque>
#include <vector>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "ssct_common/publisher/image_publisher.hpp"
#include "ssct_common/publisher/cloud_publisher.hpp"
#include "ssct_common/subscriber/image_subscriber.hpp"
#include "ssct_common/subscriber/cloud_subscriber.hpp"
#include "ssct_common/calib_param_manager.hpp"
#include "ssct_interfaces/msg/calibration_status.hpp"
#include "ssct_interfaces/msg/calibration_command.hpp"
#include "ssct_interfaces/msg/calibration_params.hpp"
#include "ssct_lidar_cam_manual_calib/image_undistorter.hpp"

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

  struct SaveRequest
  {
    bool valid{false};
    Eigen::Matrix4d transform{Eigen::Matrix4d::Identity()};
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
  bool read_data(SensorData & data);
  void clear_data();
  void process_command(const ssct_interfaces::msg::CalibrationCommand & msg);
  void process_save_requests();
  void update_status_msg(uint8_t state, const std::string & info, uint8_t progress);
  void publish_initial_params();
  void on_result_params(const ssct_interfaces::msg::CalibrationParams::SharedPtr msg);
  void save_result(const Eigen::Matrix4d & T_camera_lidar);
  bool run();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<ssct_common::CloudSubscriber<pcl::PointXYZI>> pointcloud_sub_;
  std::shared_ptr<ssct_common::ImageSubscriber> image_sub_;
  rclcpp::Subscription<ssct_interfaces::msg::CalibrationCommand>::SharedPtr cmd_sub_;
  rclcpp::Subscription<ssct_interfaces::msg::CalibrationParams>::SharedPtr result_sub_;
  rclcpp::Publisher<ssct_interfaces::msg::CalibrationStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<ssct_interfaces::msg::CalibrationParams>::SharedPtr params_pub_;
  std::shared_ptr<ssct_common::ImagePublisher> image_pub_;
  std::shared_ptr<ssct_common::CloudPublisher> cloud_pub_;
  std::shared_ptr<ImageUndistorter> image_undistorter_;
  // param
  std::string lidar_frame_id_{"lidar"};
  std::string camera_frame_id_{"camera"};
  std::string initial_calibration_file_;
  std::string output_calibration_file_;
  bool enable_compressed_image_{false};
  std::string image_topic_;
  std::string cloud_topic_;
  // sync param
  double image_time_offset_{0.0};
  double max_time_diff_{0.05};
  double capture_timeout_{5.0};
  std::chrono::steady_clock::time_point capture_start_time_;
  // calibration flow thread
  std::unique_ptr<std::thread> run_thread_;
  bool exit_{false};
  // sensor data
  size_t max_pointcloud_buffer_size_{20};
  size_t max_image_buffer_size_{10};
  std::deque<ssct_common::CloudSubscriber<pcl::PointXYZI>::MsgData> pointcloud_buffer_;
  std::map<double, ssct_common::ImageData> image_buffer_;
  // camera intrinsic data
  ssct_common::CameraIntrinsicParam camera_intrinsic_;
  // extrinsic data
  Eigen::Matrix4d T_camera_lidar_;
  Eigen::Matrix4d T_result_;
  bool has_result_{false};
  std::mutex result_mutex_;
  bool data_published_{false};
  // calibration state
  uint8_t state_;
  ssct_interfaces::msg::CalibrationStatus status_msg_;
  std::mutex status_mutex_;
  // calibration command
  std::deque<ssct_interfaces::msg::CalibrationCommand> command_msgs_;
  std::deque<SaveRequest> save_requests_;
  std::mutex command_mutex_;
};
}  // namespace ssct_lidar_cam_manual_calib
