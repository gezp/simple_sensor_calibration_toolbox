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

#include "lidar_camera_manual_calibration/calibration_node.hpp"

#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>

namespace lidar_camera_manual_calibration
{

CalibrationNode::CalibrationNode(const rclcpp::NodeOptions & options)
{
  node_ = std::make_shared<rclcpp::Node>("lidar_camera_manual_calibration_node", options);
  std::string calibrator_config;
  node_->declare_parameter("camera_frame_id", camera_frame_id_);
  node_->declare_parameter("lidar_frame_id", lidar_frame_id_);
  node_->declare_parameter("calibrator_config", calibrator_config);
  node_->declare_parameter("initial_calibration_file", initial_calibration_file_);
  node_->declare_parameter("output_calibration_file", output_calibration_file_);
  node_->declare_parameter("enable_compressed_image", enable_compressed_image_);
  node_->get_parameter("camera_frame_id", camera_frame_id_);
  node_->get_parameter("lidar_frame_id", lidar_frame_id_);
  node_->get_parameter("calibrator_config", calibrator_config);
  node_->get_parameter("initial_calibration_file", initial_calibration_file_);
  node_->get_parameter("output_calibration_file", output_calibration_file_);
  node_->get_parameter("enable_compressed_image", enable_compressed_image_);
  RCLCPP_INFO(node_->get_logger(), "calibrator_config: [%s]", calibrator_config.c_str());
  if (calibrator_config == "" || (!std::filesystem::exists(calibrator_config))) {
    RCLCPP_FATAL(node_->get_logger(), "calibrator_config is invalid");
    return;
  }
  RCLCPP_INFO(
    node_->get_logger(), "initial calibration file: [%s]", initial_calibration_file_.c_str());
  if (initial_calibration_file_ == "" || (!std::filesystem::exists(initial_calibration_file_))) {
    RCLCPP_FATAL(node_->get_logger(), "initial calibration file is invalid");
    return;
  }
  // read calibration data
  auto calibration_params = std::make_shared<calibration_common::CalibrationParams>();
  calibration_params->load(initial_calibration_file_);
  calibration_params->get_extrinsic_param(camera_frame_id_, lidar_frame_id_, T_camera_lidar_);
  std::string type;
  std::vector<double> intrinsics;
  std::vector<double> distortion_coeffs;
  calibration_params->get_camera_intrinsic_param(
    camera_frame_id_, type, intrinsics, distortion_coeffs);
  if (intrinsics.size() != 4) {
    RCLCPP_FATAL(node_->get_logger(), "camera intrinsic in calibration file is invalid");
    return;
  }
  camera_intrinsic_ = intrinsics;
  // lidar projector
  YAML::Node config_node = YAML::LoadFile(calibrator_config);
  lidar_projector_ = std::make_shared<LidarProjector>(config_node["lidar_projector"]);
  std::string topic_name_prefix =
    "/calibration/lidar_camera_manual_calibration/" + camera_frame_id_ + "/" + lidar_frame_id_;
  // pub&sub
  pointcloud_sub_ =
    std::make_shared<calibration_common::CloudSubscriber<pcl::PointXYZI>>(node_, "pointcloud", 100);
  image_sub_ = std::make_shared<calibration_common::ImageSubscriber>(
    node_, "image", 100, enable_compressed_image_);
  std::string command_topic = topic_name_prefix + "/command";
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
  status_pub_ = node_->create_publisher<calibration_interfaces::msg::CalibrationStatus>(
    topic_name_prefix + "/status", 100);
  debug_image_pub_ = std::make_shared<calibration_common::ImagePublisher>(
    node_, topic_name_prefix + "/debug_image", 100, lidar_frame_id_);
  // initialize status
  status_msg_.frame_id = camera_frame_id_;
  status_msg_.sensor_topic = image_sub_->get_topic_name();
  status_msg_.child_frame_id = lidar_frame_id_;
  status_msg_.child_sensor_topic = pointcloud_sub_->get_topic_name();
  status_msg_.calibration_type = "lidar_camera_manual_calibration";
  status_msg_.command_topic = command_topic;
  state_ = calibration_interfaces::msg::CalibrationStatus::READY;
  update_status_msg(state_, "ready to start.");
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

bool CalibrationNode::get_nearest_image(double time, double & image_timestamp, cv::Mat & image)
{
  if (image_buffer_.empty()) {
    return false;
  }
  auto cur = image_buffer_.lower_bound(time);
  if (cur == image_buffer_.end()) {
    // the last
    image_timestamp = image_buffer_.rbegin()->first;
    image = image_buffer_.rbegin()->second;
  } else if (cur == image_buffer_.begin()) {
    // the first
    image_timestamp = cur->first;
    image = cur->second;
  } else {
    // choose between prev and cur
    auto prev = std::prev(cur);
    if (fabs(prev->first - time) < fabs(cur->first - time)) {
      image_timestamp = prev->first;
      image = prev->second;
    } else {
      image_timestamp = cur->first;
      image = cur->second;
    }
  }
  return true;
}

bool CalibrationNode::read_data()
{
  pointcloud_sub_->read(pointcloud_msgs_);
  std::deque<calibration_common::ImageSubscriber::MsgData> image_msgs;
  image_sub_->read(image_msgs);
  for (auto & msg : image_msgs) {
    image_buffer_.insert({msg.time, msg.image});
    if (image_buffer_.size() > max_image_buffer_size_) {
      image_buffer_.erase(image_buffer_.begin());
    }
  }
  if (pointcloud_msgs_.empty() || image_buffer_.empty()) {
    return false;
  }
  if (pointcloud_msgs_.front().time > image_buffer_.rbegin()->first) {
    return false;
  }
  double image_timestamp;
  cv::Mat image;
  while (!pointcloud_msgs_.empty()) {
    auto & pointcloud_msg = pointcloud_msgs_.front();
    if (pointcloud_msg.time - last_sensor_time_ < 0) {
      pointcloud_msgs_.pop_front();
      continue;
    }
    // get nearest image
    if (get_nearest_image(pointcloud_msgs_.front().time, image_timestamp, image)) {
      if (fabs(pointcloud_msgs_.front().time - image_timestamp) < 0.05) {
        current_sensor_data_.time = pointcloud_msgs_.front().time;
        current_sensor_data_.pointcloud = pointcloud_msgs_.front().pointcloud;
        current_sensor_data_.image = image;
        last_sensor_time_ = current_sensor_data_.time;
        pointcloud_msgs_.pop_front();
        return true;
      }
    }
  }
  return false;
}

void CalibrationNode::clear_data()
{
  pointcloud_sub_->clear();
  image_sub_->clear();
  image_buffer_.clear();
}

bool CalibrationNode::update_calibration_params(const std::string & key, float value)
{
  if (key == "x") {
    T_camera_lidar_(0, 3) += value;
  } else if (key == "y") {
    T_camera_lidar_(1, 3) += value;
  } else if (key == "z") {
    T_camera_lidar_(2, 3) += value;
  } else if (key == "roll") {
    double v = value * 3.14159 / 180;
    T_camera_lidar_.block<3, 3>(0, 0) =
      T_camera_lidar_.block<3, 3>(0, 0) * Eigen::AngleAxisd(v, Eigen::Vector3d::UnitX());
  } else if (key == "pitch") {
    double v = value * 3.14159 / 180;
    T_camera_lidar_.block<3, 3>(0, 0) =
      T_camera_lidar_.block<3, 3>(0, 0) * Eigen::AngleAxisd(v, Eigen::Vector3d::UnitY());
  } else if (key == "yaw") {
    double v = value * 3.14159 / 180;
    T_camera_lidar_.block<3, 3>(0, 0) =
      T_camera_lidar_.block<3, 3>(0, 0) * Eigen::AngleAxisd(v, Eigen::Vector3d::UnitZ());
  } else {
    return false;
  }
  return true;
}

void CalibrationNode::process_command(const calibration_interfaces::msg::CalibrationCommand & msg)
{
  if (msg.command == calibration_interfaces::msg::CalibrationCommand::RESET) {
    state_ = calibration_interfaces::msg::CalibrationStatus::READY;
    update_status_msg(state_, "ready to start.");
  } else if (msg.command == calibration_interfaces::msg::CalibrationCommand::START) {
    state_ = calibration_interfaces::msg::CalibrationStatus::DONE;
    clear_data();
    update_status_msg(state_, "ready to project lidar pointcloud.");
  } else if (msg.command == calibration_interfaces::msg::CalibrationCommand::SAVE_RESULT) {
    save_result();
  } else if (msg.command == calibration_interfaces::msg::CalibrationCommand::COLLECT_ONCE) {
    clear_data();
    state_ = calibration_interfaces::msg::CalibrationStatus::COLLECTING;
    update_status_msg(state_, "ready to collect one lidar pointcloud.");
  } else if (msg.command == calibration_interfaces::msg::CalibrationCommand::CUSTOM_KEY_VAULE) {
    // update calibration data
    if (update_calibration_params(msg.key, msg.value)) {
      need_optimize_once_ = true;
    } else {
      RCLCPP_FATAL(node_->get_logger(), "undefined custom key value command: %s", msg.key.c_str());
    }
  } else {
    RCLCPP_FATAL(node_->get_logger(), "undefined calibration command: %d", msg.command);
  }
}

void CalibrationNode::update_status_msg(uint8_t state, const std::string & info)
{
  std::lock_guard<std::mutex> lock(status_mutex_);
  status_msg_.timestamp = node_->get_clock()->now();
  status_msg_.state = state;
  status_msg_.info = info;
}

void CalibrationNode::save_result()
{
  if (output_calibration_file_ == "") {
    RCLCPP_FATAL(node_->get_logger(), "failed to save result, no output_calibration_file!");
    return;
  }
  //
  auto calibration_params = std::make_shared<calibration_common::CalibrationParams>();
  if (std::filesystem::exists(output_calibration_file_)) {
    if (!calibration_params->load(output_calibration_file_)) {
      RCLCPP_FATAL(
        node_->get_logger(), "failed to load existed calibration data, %s",
        calibration_params->error_message().c_str());
      return;
    }
  }
  calibration_params->add_extrinsic_param(camera_frame_id_, lidar_frame_id_, T_camera_lidar_);
  if (calibration_params->save(output_calibration_file_)) {
    RCLCPP_INFO(
      node_->get_logger(), "successed to save result: %s", output_calibration_file_.c_str());
  } else {
    RCLCPP_INFO(node_->get_logger(), "failed to save result: %s", output_calibration_file_.c_str());
  }
}

bool CalibrationNode::run()
{
  // process the latest command
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (!command_msgs_.empty()) {
      auto & msg = command_msgs_.back();
      process_command(msg);
      command_msgs_.clear();
    }
  }
  // calibration flow
  if (state_ == calibration_interfaces::msg::CalibrationStatus::COLLECTING) {
    if (!read_data()) {
      return false;
    }
    state_ = calibration_interfaces::msg::CalibrationStatus::OPTIMIZING;
    need_optimize_once_ = true;
    update_status_msg(state_, "ready to project one lidar pointcloud.");
  } else if (state_ == calibration_interfaces::msg::CalibrationStatus::OPTIMIZING) {
    if (need_optimize_once_ && current_sensor_data_.time >= 0) {
      // project pointcloud
      cv::Mat img = current_sensor_data_.image.clone();
      if (lidar_projector_->project(
          *current_sensor_data_.pointcloud, camera_intrinsic_, T_camera_lidar_, img))
      {
        debug_image_pub_->publish(img, current_sensor_data_.time);
        std::stringstream ss;
        ss << T_camera_lidar_;
        update_status_msg(state_, "project one lidar frame:\n" + ss.str());
      }
      need_optimize_once_ = false;
    }
  } else if (state_ == calibration_interfaces::msg::CalibrationStatus::DONE) {
    if (!read_data()) {
      return false;
    }
    cv::Mat img = current_sensor_data_.image.clone();
    if (!lidar_projector_->project(
        *current_sensor_data_.pointcloud, camera_intrinsic_, T_camera_lidar_, img))
    {
      RCLCPP_WARN(node_->get_logger(), "failed to project");
    }
    debug_image_pub_->publish(img, current_sensor_data_.time);
    // project pointcloud
    update_status_msg(state_, "project lidar pointcloud.");
  } else {
    return false;
  }
  return true;
}

}  // namespace lidar_camera_manual_calibration
