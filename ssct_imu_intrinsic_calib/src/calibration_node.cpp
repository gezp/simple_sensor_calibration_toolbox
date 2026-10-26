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

#include "ssct_imu_intrinsic_calib/calibration_node.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <sstream>

#include "ssct_common/calib_param_manager.hpp"
#include "ssct_imu_intrinsic_calib/allan_calib/allan_calibrator.hpp"
#include "ssct_imu_intrinsic_calib/imu_tk_calib/imu_tk_calibrator.hpp"

namespace ssct_imu_intrinsic_calib
{

CalibrationNode::CalibrationNode(const rclcpp::NodeOptions & options)
{
  node_ = std::make_shared<rclcpp::Node>("ssct_imu_intrinsic_calib_node", options);
  node_->declare_parameter("imu_frame_id", frame_id_);
  node_->declare_parameter("calibration_config", calibration_config_);
  node_->declare_parameter("output_file", output_file_);
  node_->declare_parameter("autostart", autostart_);
  node_->get_parameter("imu_frame_id", frame_id_);
  node_->get_parameter("calibration_config", calibration_config_);
  node_->get_parameter("output_file", output_file_);
  node_->get_parameter("autostart", autostart_);

  if (calibration_config_.empty() || !std::filesystem::exists(calibration_config_)) {
    RCLCPP_FATAL(node_->get_logger(), "calibration_config is invalid");
    return;
  }
  try {
    const YAML::Node config = YAML::LoadFile(calibration_config_);
    if (!config["calibrator_type"]) {
      RCLCPP_FATAL(node_->get_logger(), "missing calibrator_type in calibration config");
      return;
    }
    const std::string calibrator_type = config["calibrator_type"].as<std::string>();
    if (calibrator_type == "allan_calibrator") {
      calibrator_ = std::make_shared<AllanCalibrator>(
        config["allan_calibrator"]);
    } else if (calibrator_type == "imu_tk_calibrator") {
      auto calibrator = std::make_shared<ImuTkCalibrator>(
        config["imu_tk_calibrator"]);
      calibrator->set_frame_id(frame_id_);
      calibrator_ = calibrator;
    } else {
      RCLCPP_FATAL(
        node_->get_logger(), "unsupported calibrator_type: %s", calibrator_type.c_str());
      return;
    }
  } catch (const std::exception & e) {
    RCLCPP_FATAL(node_->get_logger(), "failed to load calibration config: %s", e.what());
    return;
  }

  const std::string calibration_type = "ssct_imu_intrinsic_calib";
  const std::string topic_prefix = "/calibration/" + calibration_type + "/" + frame_id_;
  const std::string command_topic = topic_prefix + "/command";
  const std::string status_topic = topic_prefix + "/status";
  const std::string debug_topic = topic_prefix + "/debug_image";
  imu_sub_ = std::make_shared<ssct_common::ImuSubscriber>(node_, "imu", 5000);
  debug_image_pub_ =
    std::make_shared<ssct_common::ImagePublisher>(node_, debug_topic, 10, frame_id_);
  cmd_sub_ = node_->create_subscription<ssct_interfaces::msg::CalibrationCommand>(
    command_topic, 10,
    [this](const ssct_interfaces::msg::CalibrationCommand::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(command_mutex_);
      command_msgs_.push_back(*msg);
      if (command_msgs_.size() > 1) {
        command_msgs_.pop_front();
      }
    });
  status_pub_ = node_->create_publisher<ssct_interfaces::msg::CalibrationStatus>(
    status_topic, 100);

  status_msg_.frame_id = frame_id_;
  status_msg_.sensor_topic = imu_sub_->get_topic_name();
  status_msg_.calibration_type = calibration_type;
  status_msg_.command_topic = command_topic;
  if (autostart_) {
    process_command(ssct_interfaces::msg::CalibrationCommand::START);
  } else {
    process_command(ssct_interfaces::msg::CalibrationCommand::RESET);
  }

  using namespace std::chrono_literals;
  timer_ = node_->create_wall_timer(
    100ms, [this]() {
      std::lock_guard<std::mutex> lock(status_mutex_);
      status_pub_->publish(status_msg_);
    });
  run_thread_ = std::make_unique<std::thread>(
    [this]() {
      while (!exit_) {
        if (!run()) {
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

bool CalibrationNode::read_data(std::vector<ssct_common::ImuData> & samples)
{
  std::deque<ssct_common::ImuData> buffer;
  imu_sub_->read(buffer);
  samples.clear();
  for (const auto & sample : buffer) {
    if (has_data_ && sample.time <= last_time_) {
      continue;
    }
    samples.push_back(sample);
    last_time_ = sample.time;
    has_data_ = true;
  }
  return !samples.empty();
}

void CalibrationNode::clear_data()
{
  imu_sub_->clear();
  last_time_ = -1.0;
  has_data_ = false;
  calibrator_->reset();
}

void CalibrationNode::publish_debug_image()
{
  if (!debug_image_pub_ || !debug_image_pub_->has_subscribers() || !has_data_) {
    return;
  }
  const cv::Mat image = calibrator_->get_new_debug_image();
  if (!image.empty()) {
    debug_image_pub_->publish(image, last_time_);
  }
}

bool CalibrationNode::save_result()
{
  if (output_file_.empty()) {
    RCLCPP_FATAL(node_->get_logger(), "output_file is empty");
    return false;
  }
  ssct_common::CalibParamManager manager;
  ssct_common::ImuIntrinsicParam param = calibrator_->get_result();
  param.frame_id = frame_id_;
  if (!manager.add_imu_intrinsic_param(param)) {
    RCLCPP_FATAL(node_->get_logger(), "failed to add IMU intrinsic result");
    return false;
  }
  if (!manager.save(output_file_)) {
    RCLCPP_FATAL(node_->get_logger(), "failed to save result: %s", output_file_.c_str());
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "saved result: %s", output_file_.c_str());
  return true;
}

void CalibrationNode::process_command(uint8_t command)
{
  if (command == ssct_interfaces::msg::CalibrationCommand::RESET) {
    state_ = ssct_interfaces::msg::CalibrationStatus::READY;
    clear_data();
    update_status_msg(state_, "ready.", 0);
  } else if (command == ssct_interfaces::msg::CalibrationCommand::START) {
    state_ = ssct_interfaces::msg::CalibrationStatus::CALIBRATING;
    finished_ = false;
    clear_data();
    update_status_msg(
      state_, calibrator_->message(), calibrator_->progress());
  } else if (command == ssct_interfaces::msg::CalibrationCommand::PAUSE) {
    // Automatic for every calibrator; nothing to do.
  } else if (command == ssct_interfaces::msg::CalibrationCommand::SAVE) {
    if (state_ != ssct_interfaces::msg::CalibrationStatus::SUCCESSED) {
      update_status_msg(state_, "save ignored: calibration has not succeeded.", 100);
      return;
    }
    if (save_result()) {
      finished_ = true;
      update_status_msg(state_, "saved IMU intrinsic result.", 100);
    } else {
      update_status_msg(state_, "failed to save IMU intrinsic result.", 100);
    }
  } else {
    RCLCPP_FATAL(node_->get_logger(), "undefined calibration command: %d", command);
  }
}

void CalibrationNode::update_status_msg(
  uint8_t state, const std::string & info, uint8_t progress)
{
  std::lock_guard<std::mutex> lock(status_mutex_);
  status_msg_.timestamp = node_->get_clock()->now();
  status_msg_.state = state;
  status_msg_.progress = progress;
  status_msg_.info = info;
}

bool CalibrationNode::run()
{
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (!command_msgs_.empty()) {
      const auto command = command_msgs_.back().command;
      command_msgs_.clear();
      process_command(command);
    }
  }
  if (finished_ || state_ != ssct_interfaces::msg::CalibrationStatus::CALIBRATING) {
    imu_sub_->clear();
    return false;
  }
  std::vector<ssct_common::ImuData> samples;
  if (!read_data(samples)) {
    return false;
  }
  bool updated = calibrator_->add_data(samples);
  if (calibrator_->is_ready()) {
    const bool success = calibrator_->optimize();
    state_ = success ? ssct_interfaces::msg::CalibrationStatus::SUCCESSED :
      ssct_interfaces::msg::CalibrationStatus::FAILED;
    finished_ = true;
    updated = true;
  }
  update_status_msg(
    state_, calibrator_->message(), calibrator_->progress());
  if (updated) {
    publish_debug_image();
  }
  return true;
}

}  // namespace ssct_imu_intrinsic_calib
