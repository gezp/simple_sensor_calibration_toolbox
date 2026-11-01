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

#include "ssct_cam_imu_calib/calibration_node.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>

#include "ssct_cam_imu_calib/handeye_calib/handeye_calibrator.hpp"
#include "ssct_cam_imu_calib/kalibr_calib/kalibr_calibrator.hpp"

namespace ssct_cam_imu_calib
{
CalibrationNode::CalibrationNode(const rclcpp::NodeOptions & options)
{
  node_ = std::make_shared<rclcpp::Node>("ssct_cam_imu_calib_node", options);
  node_->declare_parameter("camera_frame_id", camera_frame_id_);
  node_->declare_parameter("imu_frame_id", imu_frame_id_);
  node_->declare_parameter("calibration_config", calibration_config_);
  node_->declare_parameter("initial_calibration_file", initial_calibration_file_);
  node_->declare_parameter("output_file", output_file_);
  node_->declare_parameter("enable_compressed_image", enable_compressed_image_);
  node_->declare_parameter("autostart", autostart_);
  node_->get_parameter("camera_frame_id", camera_frame_id_);
  node_->get_parameter("imu_frame_id", imu_frame_id_);
  node_->get_parameter("calibration_config", calibration_config_);
  node_->get_parameter("initial_calibration_file", initial_calibration_file_);
  node_->get_parameter("output_file", output_file_);
  node_->get_parameter("enable_compressed_image", enable_compressed_image_);
  node_->get_parameter("autostart", autostart_);

  if (calibration_config_.empty() || !std::filesystem::exists(calibration_config_)) {
    RCLCPP_FATAL(node_->get_logger(), "calibration_config is invalid");
    return;
  }
  if (initial_calibration_file_.empty() ||
    !std::filesystem::exists(initial_calibration_file_))
  {
    RCLCPP_FATAL(node_->get_logger(), "initial_calibration_file is invalid");
    return;
  }
  if (!initial_params_.load(initial_calibration_file_)) {
    RCLCPP_FATAL(
      node_->get_logger(), "failed to load initial calibration: %s",
      initial_params_.error_message().c_str());
    return;
  }
  if (!initial_params_.get_camera_intrinsic_param(camera_frame_id_, camera_intrinsic_)) {
    RCLCPP_FATAL(node_->get_logger(), "no camera intrinsic for %s", camera_frame_id_.c_str());
    return;
  }
  if (!initial_params_.get_imu_intrinsic_param(imu_frame_id_, imu_intrinsic_)) {
    RCLCPP_FATAL(node_->get_logger(), "no IMU intrinsic for %s", imu_frame_id_.c_str());
    return;
  }

  YAML::Node config;
  try {
    config = YAML::LoadFile(calibration_config_);
    const std::string calibrator_type =
      config["calibrator_type"] ? config["calibrator_type"].as<std::string>() :
      "kalibr_calibrator";
    YAML::Node calibrator_config = config[calibrator_type];
    if (calibrator_type == "handeye_calibrator") {
      auto calibrator = std::make_shared<HandEyeCalibrator>(calibrator_config);
      calibrator->set_camera_intrinsic_param(camera_intrinsic_);
      calibrator_ = calibrator;
    } else if (calibrator_type == "kalibr_calibrator") {
      auto calibrator = std::make_shared<KalibrCalibrator>(calibrator_config);
      calibrator->set_camera_intrinsic_param(camera_intrinsic_);
      calibrator->set_imu_intrinsic_param(imu_intrinsic_);
      calibrator_ = calibrator;
    } else {
      RCLCPP_FATAL(
        node_->get_logger(), "unsupported calibrator_type: %s",
        calibrator_type.c_str());
      return;
    }
  } catch (const std::exception & e) {
    RCLCPP_FATAL(node_->get_logger(), "failed to init calibrator: %s", e.what());
    return;
  }

  const std::string prefix =
    "/calibration/ssct_cam_imu_calib/" + camera_frame_id_ + "_tf_" + imu_frame_id_;
  const std::string command_topic = prefix + "/command";
  const std::string status_topic = prefix + "/status";
  const std::string debug_topic = prefix + "/debug_image";

  image_sub_ =
    std::make_shared<ssct_common::ImageSubscriber>(
    node_, "image", 100, enable_compressed_image_);
  imu_sub_ = std::make_shared<ssct_common::ImuSubscriber>(node_, "imu", 5000);
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
  debug_image_pub_ =
    std::make_shared<ssct_common::ImagePublisher>(
    node_, debug_topic, 10, camera_frame_id_);

  status_msg_.frame_id = camera_frame_id_;
  status_msg_.sensor_topic = image_sub_->get_topic_name();
  status_msg_.child_frame_id = imu_frame_id_;
  status_msg_.child_sensor_topic = imu_sub_->get_topic_name();
  status_msg_.calibration_type = "ssct_cam_imu_calib";
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

bool CalibrationNode::read_data(
  std::vector<ssct_common::ImageData> & images,
  std::vector<ssct_common::ImuData> & imu)
{
  std::deque<ssct_common::ImuData> imu_buffer;
  imu_sub_->read(imu_buffer);
  imu.assign(imu_buffer.begin(), imu_buffer.end());

  std::deque<ssct_common::ImageData> image_buffer;
  image_sub_->read(image_buffer);
  images.assign(image_buffer.begin(), image_buffer.end());
  if (!images.empty()) {
    last_image_time_ = images.back().time;
  }
  return !images.empty() || !imu.empty();
}

void CalibrationNode::clear_data()
{
  image_sub_->clear();
  imu_sub_->clear();
  last_image_time_ = -1.0;
  calibrator_->reset();
}

void CalibrationNode::publish_debug_image()
{
  if (!debug_image_pub_->has_subscribers() || last_image_time_ < 0.0) {
    return;
  }
  const cv::Mat image = calibrator_->get_new_debug_image();
  if (!image.empty()) {
    debug_image_pub_->publish(image, last_image_time_);
  }
}

void CalibrationNode::process_command(uint8_t command)
{
  if (command == ssct_interfaces::msg::CalibrationCommand::RESET) {
    finished_ = false;
    state_ = ssct_interfaces::msg::CalibrationStatus::READY;
    clear_data();
    update_status_msg(state_, "ready.", 0);
  } else if (command == ssct_interfaces::msg::CalibrationCommand::START) {
    finished_ = false;
    state_ = ssct_interfaces::msg::CalibrationStatus::CALIBRATING;
    clear_data();
    update_status_msg(state_, calibrator_->message(), 0);
  } else if (command == ssct_interfaces::msg::CalibrationCommand::PAUSE) {
    // Automatic for every calibrator; nothing to do.
  } else if (command == ssct_interfaces::msg::CalibrationCommand::SAVE) {
    if (state_ != ssct_interfaces::msg::CalibrationStatus::SUCCESSED) {
      update_status_msg(state_, "save ignored: calibration has not succeeded.", 100);
      return;
    }
    if (save_result()) {
      finished_ = true;
      update_status_msg(state_, "saved calibration result.", 100);
    } else {
      update_status_msg(state_, "failed to save calibration result.", 100);
    }
  } else {
    RCLCPP_FATAL(node_->get_logger(), "undefined calibration command: %d", command);
  }
}

bool CalibrationNode::save_result()
{
  if (output_file_.empty()) {
    RCLCPP_FATAL(node_->get_logger(), "output_file is empty");
    return false;
  }
  ssct_common::CalibParamManager manager = initial_params_;
  if (std::filesystem::exists(output_file_)) {
    ssct_common::CalibParamManager existing;
    if (existing.load(output_file_)) {
      manager = existing;
    }
  }
  manager.add_camera_intrinsic_param(camera_intrinsic_);
  manager.add_imu_intrinsic_param(imu_intrinsic_);
  ssct_common::ExtrinsicParam extrinsic = calibrator_->get_result();
  extrinsic.frame_id = camera_frame_id_;
  extrinsic.child_frame_id = imu_frame_id_;
  if (!manager.add_extrinsic_param(extrinsic)) {
    RCLCPP_FATAL(node_->get_logger(), "failed to add extrinsic result");
    return false;
  }
  if (!manager.save(output_file_)) {
    RCLCPP_FATAL(node_->get_logger(), "failed to save result: %s", output_file_.c_str());
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "saved result: %s", output_file_.c_str());
  return true;
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
    image_sub_->clear();
    return false;
  }
  std::vector<ssct_common::ImageData> images;
  std::vector<ssct_common::ImuData> imu;
  if (!read_data(images, imu)) {
    return false;
  }
  calibrator_->add_data(images, imu);
  if (calibrator_->ready_to_optimize()) {
    update_status_msg(
      state_, "optimizing...", calibrator_->progress());
    const bool success = calibrator_->optimize();
    state_ = success ? ssct_interfaces::msg::CalibrationStatus::SUCCESSED :
      ssct_interfaces::msg::CalibrationStatus::FAILED;
    finished_ = true;
  }
  update_status_msg(state_, calibrator_->message(), calibrator_->progress());
  if (!images.empty()) {
    publish_debug_image();
  }
  return true;
}

}  // namespace ssct_cam_imu_calib
