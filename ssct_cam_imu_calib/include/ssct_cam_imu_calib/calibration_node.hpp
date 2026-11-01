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
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "ssct_cam_imu_calib/cam_imu_calibrator_interface.hpp"
#include "ssct_common/calib_param/camera_intrinsic_param.hpp"
#include "ssct_common/calib_param/imu_intrinsic_param.hpp"
#include "ssct_common/calib_param_manager.hpp"
#include "ssct_common/publisher/image_publisher.hpp"
#include "ssct_common/sensor_data/image_data.hpp"
#include "ssct_common/sensor_data/imu_data.hpp"
#include "ssct_common/subscriber/image_subscriber.hpp"
#include "ssct_common/subscriber/imu_subscriber.hpp"
#include "ssct_interfaces/msg/calibration_command.hpp"
#include "ssct_interfaces/msg/calibration_status.hpp"

namespace ssct_cam_imu_calib
{

class CalibrationNode
{
public:
  explicit CalibrationNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CalibrationNode();
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
  {
    return node_->get_node_base_interface();
  }

private:
  bool read_data(
    std::vector<ssct_common::ImageData> & images,
    std::vector<ssct_common::ImuData> & imu);
  void clear_data();
  bool save_result();
  void publish_debug_image();
  void process_command(uint8_t command);
  void update_status_msg(uint8_t state, const std::string & info, uint8_t progress);
  bool run();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<ssct_common::ImageSubscriber> image_sub_;
  std::shared_ptr<ssct_common::ImuSubscriber> imu_sub_;
  rclcpp::Subscription<ssct_interfaces::msg::CalibrationCommand>::SharedPtr cmd_sub_;
  rclcpp::Publisher<ssct_interfaces::msg::CalibrationStatus>::SharedPtr status_pub_;
  std::shared_ptr<ssct_common::ImagePublisher> debug_image_pub_;
  std::shared_ptr<CamImuCalibratorInterface> calibrator_;
  // config
  std::string camera_frame_id_{"camera"};
  std::string imu_frame_id_{"imu"};
  std::string calibration_config_;
  std::string initial_calibration_file_;
  std::string output_file_;
  bool enable_compressed_image_{false};
  bool autostart_{false};
  ssct_common::CameraIntrinsicParam camera_intrinsic_;
  ssct_common::ImuIntrinsicParam imu_intrinsic_;
  ssct_common::CalibParamManager initial_params_;
  // latest image timestamp, used to stamp the debug image
  double last_image_time_{-1.0};
  // calibration flow thread
  std::unique_ptr<std::thread> run_thread_;
  std::atomic<bool> exit_{false};
  bool finished_{false};
  uint8_t state_{ssct_interfaces::msg::CalibrationStatus::READY};
  ssct_interfaces::msg::CalibrationStatus status_msg_;
  std::mutex status_mutex_;
  //
  std::deque<ssct_interfaces::msg::CalibrationCommand> command_msgs_;
  std::mutex command_mutex_;
};

}  // namespace ssct_cam_imu_calib
