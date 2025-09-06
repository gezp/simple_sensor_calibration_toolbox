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

#include "camera_intrinsic_calibration/calibration_node.hpp"

#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>

namespace camera_intrinsic_calibration
{

CalibrationNode::CalibrationNode(const rclcpp::NodeOptions & options)
{
  node_ = std::make_shared<rclcpp::Node>("camera_intrinsic_calibration_node", options);
  std::string calibrator_config;
  node_->declare_parameter("frame_id", frame_id_);
  node_->declare_parameter("output_file", output_file_);
  node_->declare_parameter("calibrator_config", calibrator_config);
  node_->declare_parameter("enable_compressed_image", enable_compressed_image_);
  node_->declare_parameter("autostart", autostart_);
  node_->get_parameter("frame_id", frame_id_);
  node_->get_parameter("output_file", output_file_);
  node_->get_parameter("calibrator_config", calibrator_config);
  node_->get_parameter("enable_compressed_image", enable_compressed_image_);
  node_->get_parameter("autostart", autostart_);
  RCLCPP_INFO(node_->get_logger(), "calibrator_config: [%s]", calibrator_config.c_str());
  if (calibrator_config == "" || (!std::filesystem::exists(calibrator_config))) {
    RCLCPP_FATAL(node_->get_logger(), "calibrator_config is invalid");
    return;
  }
  // pub&sub
  image_sub_ = std::make_shared<calibration_common::ImageSubscriber>(
    node_, "image", 100, enable_compressed_image_);
  std::string command_topic = "/calibration/camera_intrinsic/" + frame_id_ + "/command";
  auto command_msg_callback =
    [this](const calibration_interfaces::msg::CalibrationCommand::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(command_mutex_);
      command_msgs_.push_back(*msg);
      if (command_msgs_.size() > 1) {
        command_msgs_.pop_front();
      }
    };
  cmd_sub_ = node_->create_subscription<calibration_interfaces::msg::CalibrationCommand>(
    command_topic, 10, command_msg_callback);
  std::string status_topic = "/calibration/camera_intrinsic/" + frame_id_ + "/status";
  status_pub_ =
    node_->create_publisher<calibration_interfaces::msg::CalibrationStatus>(status_topic, 100);
  std::string debug_topic = "/calibration/camera_intrinsic/" + frame_id_ + "/debug_image";
  debug_image_pub_ =
    std::make_shared<calibration_common::ImagePublisher>(node_, debug_topic, 100, frame_id_);

  // calibrator
  YAML::Node config_node = YAML::LoadFile(calibrator_config);
  calibrator_ = std::make_shared<PinholeCalibrator>(config_node["pinhole_calibrator"]);
  calibration_params_ = std::make_shared<calibration_common::CalibrationParams>();
  // initialize status
  status_msg_.frame_id = frame_id_;
  status_msg_.sensor_topic = image_sub_->get_topic_name();
  status_msg_.calibration_type = "camera_intrinsic_calibration";
  status_msg_.command_topic = command_topic;
  if (autostart_) {
    process_command(calibration_interfaces::msg::CalibrationCommand::START);
  } else {
    process_command(calibration_interfaces::msg::CalibrationCommand::RESET);
  }
  // status timer
  using namespace std::chrono_literals;
  timer_ = node_->create_wall_timer(
    100ms, [this]() {
      std::lock_guard<std::mutex> lock(status_mutex_);
      status_pub_->publish(status_msg_);
    });
  // thread
  run_thread_ = std::make_unique<std::thread>(
    [this]() {
      while (!exit_) {
        if (!run()) {
          using namespace std::chrono_literals;
          std::this_thread::sleep_for(50ms);
        }
      }
    });
}

CalibrationNode::~CalibrationNode()
{
  exit_ = true;
  if (run_thread_) {
    run_thread_->join();
  }
}

void CalibrationNode::read_data()
{
  std::deque<calibration_common::ImageSubscriber::MsgData> msg_buffer;
  image_sub_->read(msg_buffer);
  for (auto & msg : msg_buffer) {
    if (sensor_data_buffer_.empty() || msg.time - sensor_data_buffer_.back().time > 0.5) {
      SensorData data;
      data.time = msg.time;
      data.image = msg.image;
      sensor_data_buffer_.push_back(data);
    }
  }
}

void CalibrationNode::clear_data()
{
  image_sub_->clear();
  sensor_data_buffer_.clear();
}

void CalibrationNode::process_command(uint8_t command)
{
  if (command == calibration_interfaces::msg::CalibrationCommand::RESET) {
    state_ = calibration_interfaces::msg::CalibrationStatus::READY;
    calibrator_->clear();
    update_status_msg(state_, "ready.");
  } else if (command == calibration_interfaces::msg::CalibrationCommand::START) {
    is_auto_mode_ = true;
    state_ = calibration_interfaces::msg::CalibrationStatus::COLLECTING;
    clear_data();
    update_status_msg(state_, "start to collect data.");
  } else if (command == calibration_interfaces::msg::CalibrationCommand::SAVE_RESULT) {
    if (state_ == calibration_interfaces::msg::CalibrationStatus::DONE && success_) {
      save_result();
    } else {
      RCLCPP_FATAL(
        node_->get_logger(),
        "failed to save result, need be state DONE and calibrated successfully!");
      return;
    }
  } else if (command == calibration_interfaces::msg::CalibrationCommand::COLLECT_ONCE) {
    is_auto_mode_ = false;
    state_ = calibration_interfaces::msg::CalibrationStatus::COLLECTING;
    clear_data();
    need_collect_once_ = true;
  } else if (command == calibration_interfaces::msg::CalibrationCommand::OPTIMIZE_ONCE) {
    is_auto_mode_ = false;
    state_ = calibration_interfaces::msg::CalibrationStatus::OPTIMIZING;
    need_optimize_once_ = true;
  } else {
    RCLCPP_FATAL(node_->get_logger(), "undefined calibration command: %d", command);
  }
}

void CalibrationNode::update_status_msg(uint8_t state, const std::string & info, bool success)
{
  std::lock_guard<std::mutex> lock(status_mutex_);
  status_msg_.timestamp = node_->get_clock()->now();
  status_msg_.state = state;
  status_msg_.success = success;
  status_msg_.info = info;
}

void CalibrationNode::save_result()
{
  if (output_file_ == "") {
    RCLCPP_FATAL(node_->get_logger(), "failed to save result, no output_file");
    return;
  }
  if (std::filesystem::exists(output_file_)) {
    if (!calibration_params_->load(output_file_)) {
      RCLCPP_FATAL(
        node_->get_logger(), "failed to load existed calibration data, %s",
        calibration_params_->error_message().c_str());
      return;
    }
  }
  calibration_params_->add_camera_intrinsic_param(
    frame_id_, calibrator_->get_type(), calibrator_->get_intrinsics(),
    calibrator_->get_distortion_coeffs());
  if (calibration_params_->save(output_file_)) {
    RCLCPP_INFO(node_->get_logger(), "successed to save result: %s", output_file_.c_str());
  } else {
    RCLCPP_INFO(node_->get_logger(), "failed to save result: %s", output_file_.c_str());
  }
}

bool CalibrationNode::run()
{
  // process the latest command
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (!command_msgs_.empty()) {
      auto & msg = command_msgs_.back();
      process_command(msg.command);
      command_msgs_.clear();
    }
  }
  // calibration flow
  if (state_ == calibration_interfaces::msg::CalibrationStatus::COLLECTING) {
    read_data();
    if (sensor_data_buffer_.empty()) {
      return false;
    }
    auto sensor_data = sensor_data_buffer_.front();
    sensor_data_buffer_.pop_front();
    if (is_auto_mode_ || need_collect_once_) {
      if (calibrator_->process_image(sensor_data.image)) {
        // debug
        debug_image_pub_->publish(calibrator_->get_debug_image(), sensor_data.time);
      }
      update_status_msg(state_, calibrator_->get_status_message());
      need_collect_once_ = false;
    }
    if (is_auto_mode_ && calibrator_->ready_to_optimize()) {
      state_ = calibration_interfaces::msg::CalibrationStatus::OPTIMIZING;
    }
  } else if (state_ == calibration_interfaces::msg::CalibrationStatus::OPTIMIZING) {
    if (is_auto_mode_ || need_optimize_once_) {
      update_status_msg(state_, "start to optimize.");
      success_ = calibrator_->optimize();
      state_ = calibration_interfaces::msg::CalibrationStatus::DONE;
      update_status_msg(state_, calibrator_->get_status_message(), success_);
      need_optimize_once_ = false;
    }
  } else {
    return false;
  }
  return true;
}

}  // namespace camera_intrinsic_calibration
