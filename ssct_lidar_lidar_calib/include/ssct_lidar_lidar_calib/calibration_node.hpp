// Copyright 2026 Gezp (https://github.com/gezp).
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

#include <atomic>
#include <chrono>
#include <deque>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "ssct_common/calib_param_manager.hpp"
#include "ssct_common/publisher/cloud_publisher.hpp"
#include "ssct_common/subscriber/cloud_subscriber.hpp"
#include "ssct_interfaces/msg/calibration_command.hpp"
#include "ssct_interfaces/msg/calibration_params.hpp"
#include "ssct_interfaces/msg/calibration_status.hpp"

#include "ssct_lidar_lidar_calib/simple_calibrator.hpp"

namespace ssct_lidar_lidar_calib
{

class CalibrationNode
{
  using CloudData = ssct_common::CloudSubscriber<pcl::PointXYZI>::MsgData;

  struct SensorData
  {
    double time;
    pcl::PointCloud<pcl::PointXYZI>::Ptr target;
    pcl::PointCloud<pcl::PointXYZI>::Ptr source;
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
  bool read_data(SensorData & data);
  void clear_data();
  void process_command(const ssct_interfaces::msg::CalibrationCommand & msg);
  void process_save_requests();
  void update_status_msg(uint8_t state, const std::string & info, uint8_t progress);
  void publish_initial_params();
  void publish_result_params(const Eigen::Matrix4d & T_target_source);
  void save_result(const Eigen::Matrix4d & T_target_source);
  bool run();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<ssct_common::CloudSubscriber<pcl::PointXYZI>> target_sub_;
  std::shared_ptr<ssct_common::CloudSubscriber<pcl::PointXYZI>> source_sub_;
  rclcpp::Subscription<ssct_interfaces::msg::CalibrationCommand>::SharedPtr cmd_sub_;
  rclcpp::Publisher<ssct_interfaces::msg::CalibrationStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<ssct_interfaces::msg::CalibrationParams>::SharedPtr params_pub_;
  rclcpp::Publisher<ssct_interfaces::msg::CalibrationParams>::SharedPtr result_pub_;
  std::shared_ptr<ssct_common::CloudPublisher> target_pub_;
  std::shared_ptr<ssct_common::CloudPublisher> source_pub_;
  // param
  std::string target_frame_id_{"target_lidar"};
  std::string source_frame_id_{"source_lidar"};
  std::string initial_calibration_file_;
  std::string output_calibration_file_;
  std::string target_topic_;
  std::string source_topic_;
  // sync param
  double source_time_offset_{0.0};
  double max_time_diff_{0.05};
  double capture_timeout_{5.0};
  std::chrono::steady_clock::time_point capture_start_time_;
  // calibration flow thread
  std::unique_ptr<std::thread> run_thread_;
  std::atomic<bool> exit_{false};
  // sensor data
  size_t max_pointcloud_buffer_size_{20};
  std::deque<CloudData> target_buffer_;
  std::deque<CloudData> source_buffer_;
  // extrinsic data
  Eigen::Matrix4d T_target_source_;
  Eigen::Matrix4d T_result_;
  bool has_result_{false};
  std::mutex result_mutex_;
  bool data_processed_{false};
  // calibration state
  uint8_t state_;
  ssct_interfaces::msg::CalibrationStatus status_msg_;
  std::mutex status_mutex_;
  // calibration command
  std::deque<ssct_interfaces::msg::CalibrationCommand> command_msgs_;
  std::deque<SaveRequest> save_requests_;
  std::mutex command_mutex_;
  // auto calibrator
  std::unique_ptr<SimpleCalibrator> calibrator_;
};

}  // namespace ssct_lidar_lidar_calib
