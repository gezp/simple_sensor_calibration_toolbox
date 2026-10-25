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

#include "ssct_camera_intrinsic_calib/calibration_node.hpp"

#include <yaml-cpp/yaml.h>
#include <array>
#include <filesystem>
#include <fstream>

namespace ssct_camera_intrinsic_calib
{

CalibrationNode::CalibrationNode(const rclcpp::NodeOptions & options)
{
  node_ = std::make_shared<rclcpp::Node>("ssct_camera_intrinsic_calib_node", options);
  std::string calibration_config;
  node_->declare_parameter("frame_id", frame_id_);
  node_->declare_parameter("output_file", output_file_);
  node_->declare_parameter("calibration_config", calibration_config);
  node_->declare_parameter("enable_compressed_image", enable_compressed_image_);
  node_->declare_parameter("autostart", autostart_);
  node_->get_parameter("frame_id", frame_id_);
  node_->get_parameter("output_file", output_file_);
  node_->get_parameter("calibration_config", calibration_config);
  node_->get_parameter("enable_compressed_image", enable_compressed_image_);
  node_->get_parameter("autostart", autostart_);
  RCLCPP_INFO(node_->get_logger(), "calibration_config: [%s]", calibration_config.c_str());
  if (calibration_config == "" || (!std::filesystem::exists(calibration_config))) {
    RCLCPP_FATAL(node_->get_logger(), "calibration_config is invalid");
    return;
  }
  // pub&sub
  image_sub_ =
    std::make_shared<ssct_common::ImageSubscriber>(node_, "image", 100, enable_compressed_image_);
  std::string command_topic = "/calibration/camera_intrinsic/" + frame_id_ + "/command";
  auto command_msg_callback =
    [this](const ssct_interfaces::msg::CalibrationCommand::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(command_mutex_);
      command_msgs_.push_back(*msg);
      if (command_msgs_.size() > 1) {
        command_msgs_.pop_front();
      }
    };
  cmd_sub_ = node_->create_subscription<ssct_interfaces::msg::CalibrationCommand>(
    command_topic, 10, command_msg_callback);
  std::string status_topic = "/calibration/camera_intrinsic/" + frame_id_ + "/status";
  status_pub_ = node_->create_publisher<ssct_interfaces::msg::CalibrationStatus>(status_topic, 100);
  std::string debug_topic = "/calibration/camera_intrinsic/" + frame_id_ + "/debug_image";
  debug_image_pub_ =
    std::make_shared<ssct_common::ImagePublisher>(node_, debug_topic, 100, frame_id_);

  // collector & calibrator
  YAML::Node config_node = YAML::LoadFile(calibration_config);
  try {
    std::string calibrator_type = config_node["calibrator_type"].as<std::string>();
    YAML::Node detector_config_node = config_node["chessboard_detector"];
    if (!detector_config_node) {
      RCLCPP_FATAL(node_->get_logger(), "missing chessboard_detector config");
      return;
    }
    detector_ = std::make_unique<ChessboardDetector>(detector_config_node);
    image_picker_ = std::make_unique<ImagePicker>(config_node["image_picker"]);
    if (calibrator_type == "opencv_calibrator") {
      calibrator_ = std::make_shared<OpenCVCalibrator>();
    } else if (calibrator_type == "opencv_fisheye_calibrator") {
      calibrator_ = std::make_shared<OpenCVFisheyeCalibrator>();
    } else if (calibrator_type == "opencv_omnidir_calibrator") {
      calibrator_ = std::make_shared<OpenCVOmnidirCalibrator>();
    } else {
      RCLCPP_FATAL(
        node_->get_logger(), "unsupported calibrator_type: %s", calibrator_type.c_str());
      return;
    }
  } catch (const std::exception & e) {
    RCLCPP_FATAL(node_->get_logger(), "invalid calibration config: %s", e.what());
    return;
  }
  calib_param_manager_ = std::make_shared<ssct_common::CalibParamManager>();
  // initialize status
  status_msg_.frame_id = frame_id_;
  status_msg_.sensor_topic = image_sub_->get_topic_name();
  status_msg_.calibration_type = "ssct_camera_intrinsic_calib";
  status_msg_.command_topic = command_topic;
  if (autostart_) {
    process_command(ssct_interfaces::msg::CalibrationCommand::START);
  } else {
    process_command(ssct_interfaces::msg::CalibrationCommand::RESET);
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
  std::deque<ssct_common::ImageData> msg_buffer;
  image_sub_->read(sensor_data_buffer_);
  for (auto & msg : msg_buffer) {
    if (sensor_data_buffer_.empty() || msg.time - sensor_data_buffer_.back().time > 0.5) {
      sensor_data_buffer_.push_back(msg);
    }
  }
}

void CalibrationNode::clear_data()
{
  image_sub_->clear();
  sensor_data_buffer_.clear();
}

bool CalibrationNode::process_image(const cv::Mat & image)
{
  total_img_num_++;
  if (image_size_ == cv::Size(0, 0)) {
    image_size_ = image.size();
    calibrator_->set_image_size(image_size_);
  }
  if (image.size() != image_size_) {
    collect_status_message_ =
      "[collect data] invalid image size, total image:" + std::to_string(total_img_num_);
    return false;
  }
  std::vector<cv::Point2f> image_corners;
  if (!detector_->detect(image, image_corners)) {
    collect_status_message_ = "[collect data] failed to find chessborad corners, total image:" +
      std::to_string(total_img_num_);
    return false;
  }
  const std::array<cv::Point2f, 4> board_corners =
    detector_->calculate_board_corners(image_corners);
  image_picker_->set_image_size(image.size());
  const bool accepted = image_picker_->pick(image_corners, board_corners);
  if (accepted) {
    calibrator_->add_points(image_corners, detector_->get_pattern_points());
    valid_img_num_++;
  }
  // debug
  debug_image_ = image.clone();
  cv::drawChessboardCorners(
    debug_image_, detector_->get_corner_size(), image_corners, true);
  debug_image_ = image_picker_->draw_debug(debug_image_);
  collect_status_message_ = "[collect data] total image:" + std::to_string(total_img_num_) +
    ", valid image:" + std::to_string(valid_img_num_) + ".";
  return true;
}

void CalibrationNode::clear_collection()
{
  image_size_ = cv::Size(0, 0);
  total_img_num_ = 0;
  valid_img_num_ = 0;
  collect_status_message_.clear();
  debug_image_.release();
  image_picker_->reset();
}

void CalibrationNode::process_command(uint8_t command)
{
  if (command == ssct_interfaces::msg::CalibrationCommand::RESET) {
    state_ = ssct_interfaces::msg::CalibrationStatus::READY;
    clear_collection();
    calibrator_->clear();
    update_status_msg(state_, "ready.", 0);
  } else if (command == ssct_interfaces::msg::CalibrationCommand::START) {
    state_ = ssct_interfaces::msg::CalibrationStatus::CALIBRATING;
    last_valid_image_time_ = -1.0;
    clear_data();
    update_status_msg(state_, "start to calibrating.", 1);
  } else if (command == ssct_interfaces::msg::CalibrationCommand::SAVE) {
    if (state_ != ssct_interfaces::msg::CalibrationStatus::SUCCESSED) {
      RCLCPP_FATAL(
        node_->get_logger(),
        "failed to save result, need be state DONE and calibrated successfully!");
      return;
    }
    if (save_result()) {
      update_status_msg(state_, "successed to save result.", 100);
    } else {
      update_status_msg(state_, "failed to save result.", 100);
    }
  } else {
    RCLCPP_FATAL(node_->get_logger(), "undefined calibration command: %d", command);
  }
}

void CalibrationNode::update_status_msg(uint8_t state, const std::string & info, uint8_t progress)
{
  std::lock_guard<std::mutex> lock(status_mutex_);
  status_msg_.timestamp = node_->get_clock()->now();
  status_msg_.state = state;
  status_msg_.progress = progress;
  status_msg_.info = info;
}

bool CalibrationNode::save_result()
{
  if (output_file_ == "") {
    RCLCPP_FATAL(node_->get_logger(), "failed to save result, no output_file");
    return false;
  }
  if (std::filesystem::exists(output_file_)) {
    if (!calib_param_manager_->load(output_file_)) {
      RCLCPP_FATAL(
        node_->get_logger(), "failed to load existed calibration data, %s",
        calib_param_manager_->error_message().c_str());
      return false;
    }
  }
  ssct_common::CameraIntrinsicParam param;
  param.frame_id = frame_id_;
  param.type = calibrator_->get_type();
  auto image_size = calibrator_->get_image_size();
  param.height = image_size.height;
  param.width = image_size.width;
  param.intrinsics = calibrator_->get_intrinsics();
  param.distortion_coeffs = calibrator_->get_distortion_coeffs();
  if (!calib_param_manager_->add_camera_intrinsic_param(param)) {
    RCLCPP_FATAL(
      node_->get_logger(), "failed to add camera intrinsic param, %s",
      calib_param_manager_->error_message().c_str());
    return false;
  }
  if (calib_param_manager_->save(output_file_)) {
    RCLCPP_INFO(node_->get_logger(), "successed to save result: %s", output_file_.c_str());
    return true;
  } else {
    RCLCPP_INFO(node_->get_logger(), "failed to save result: %s", output_file_.c_str());
    return false;
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
  if (state_ == ssct_interfaces::msg::CalibrationStatus::CALIBRATING) {
    if (!image_picker_->ready()) {
      read_data();
      if (sensor_data_buffer_.empty()) {
        return false;
      }
      // pick the first image at least 1s after the last valid image
      bool found = false;
      ssct_common::ImageData sensor_data;
      while (!sensor_data_buffer_.empty()) {
        sensor_data = sensor_data_buffer_.front();
        sensor_data_buffer_.pop_front();
        if (last_valid_image_time_ < 0.0 || sensor_data.time - last_valid_image_time_ > 1.0) {
          found = true;
          break;
        }
      }
      if (!found) {
        return false;
      }
      if (process_image(sensor_data.image)) {
        last_valid_image_time_ = sensor_data.time;
        // debug
        cv::Mat debug_image_rgb;
        cv::cvtColor(debug_image_, debug_image_rgb, cv::COLOR_BGR2RGB);
        debug_image_pub_->publish(debug_image_rgb, sensor_data.time);
      }
      update_status_msg(state_, collect_status_message_, 50);
    } else {
      update_status_msg(state_, "start to optimize.", 51);
      if (calibrator_->optimize()) {
        state_ = ssct_interfaces::msg::CalibrationStatus::SUCCESSED;
      } else {
        state_ = ssct_interfaces::msg::CalibrationStatus::FAILED;
      }
      update_status_msg(state_, calibrator_->get_status_message(), 100);
    }
  } else {
    return false;
  }
  return true;
}

}  // namespace ssct_camera_intrinsic_calib
