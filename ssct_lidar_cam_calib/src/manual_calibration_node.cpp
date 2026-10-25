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

#include "ssct_lidar_cam_calib/manual_calibration_node.hpp"

#include <yaml-cpp/yaml.h>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>

namespace ssct_lidar_cam_calib
{
namespace
{

ssct_interfaces::msg::ExtrinsicParam make_extrinsic_param(
  const std::string & frame_id, const std::string & child_frame_id, const Eigen::Matrix4d & T)
{
  ssct_interfaces::msg::ExtrinsicParam msg;
  msg.frame_id = frame_id;
  msg.child_frame_id = child_frame_id;
  msg.transform.translation.x = T(0, 3);
  msg.transform.translation.y = T(1, 3);
  msg.transform.translation.z = T(2, 3);
  Eigen::Quaterniond q(T.block<3, 3>(0, 0));
  q.normalize();
  msg.transform.rotation.x = q.x();
  msg.transform.rotation.y = q.y();
  msg.transform.rotation.z = q.z();
  msg.transform.rotation.w = q.w();
  return msg;
}

bool parse_extrinsic_param(const ssct_interfaces::msg::ExtrinsicParam & msg, Eigen::Matrix4d & T)
{
  Eigen::Quaterniond q(
    msg.transform.rotation.w, msg.transform.rotation.x, msg.transform.rotation.y,
    msg.transform.rotation.z);
  if (q.norm() < 1e-6) {
    return false;
  }
  q.normalize();
  T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = q.toRotationMatrix();
  if (T.block<3, 3>(0, 0).determinant() < 0.0) {
    return false;
  }
  T(0, 3) = msg.transform.translation.x;
  T(1, 3) = msg.transform.translation.y;
  T(2, 3) = msg.transform.translation.z;
  return true;
}

}  // namespace

CalibrationNode::CalibrationNode(const rclcpp::NodeOptions & options)
{
  node_ = std::make_shared<rclcpp::Node>("ssct_lidar_cam_calib_node", options);
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
  auto calib_param_manager = std::make_shared<ssct_common::CalibParamManager>();
  if (!calib_param_manager->load(initial_calibration_file_)) {
    RCLCPP_FATAL(
      node_->get_logger(), "failed to load calibration data, %s",
      calib_param_manager->error_message().c_str());
    return;
  }
  ssct_common::ExtrinsicParam extrinsic_param;
  if (!calib_param_manager->get_extrinsic_param(
      camera_frame_id_, lidar_frame_id_, extrinsic_param))
  {
    RCLCPP_FATAL(
      node_->get_logger(), "no extrinsic from %s to %s", lidar_frame_id_.c_str(),
      camera_frame_id_.c_str());
    return;
  }
  T_camera_lidar_ = extrinsic_param.transform;
  if (!calib_param_manager->get_camera_intrinsic_param(camera_frame_id_, camera_intrinsic_)) {
    RCLCPP_FATAL(
      node_->get_logger(), "no camera intrinsic for %s", camera_frame_id_.c_str());
    return;
  }
  // config
  YAML::Node config_node = YAML::LoadFile(calibrator_config);
  image_undistorter_ = std::make_shared<ImageUndistorter>();
  if (!image_undistorter_->init(camera_intrinsic_)) {
    RCLCPP_FATAL(
      node_->get_logger(), "failed to init image undistorter, %s",
      image_undistorter_->error_message().c_str());
    return;
  }
  auto sync_node = config_node["sync"];
  if (sync_node.IsDefined() && sync_node.IsMap()) {
    if (sync_node["image_time_offset"].IsDefined()) {
      image_time_offset_ = sync_node["image_time_offset"].as<double>();
    }
    if (sync_node["max_time_diff"].IsDefined()) {
      max_time_diff_ = sync_node["max_time_diff"].as<double>();
    }
    if (sync_node["timeout"].IsDefined()) {
      capture_timeout_ = sync_node["timeout"].as<double>();
    }
  }
  std::string topic_name_prefix =
    "/calibration/ssct_lidar_cam_calib/" + camera_frame_id_ + "_tf_" + lidar_frame_id_;
  image_topic_ = topic_name_prefix + "/image";
  cloud_topic_ = topic_name_prefix + "/pointcloud";
  // pub&sub
  pointcloud_sub_ =
    std::make_shared<ssct_common::CloudSubscriber<pcl::PointXYZI>>(node_, "pointcloud", 100);
  image_sub_ =
    std::make_shared<ssct_common::ImageSubscriber>(node_, "image", 100, enable_compressed_image_);
  std::string command_topic = topic_name_prefix + "/command";
  auto command_msg_callback =
    [this](const ssct_interfaces::msg::CalibrationCommand::SharedPtr msg) {
      if (msg->command == ssct_interfaces::msg::CalibrationCommand::SAVE) {
        // The check must happen when the command is received, not when it is processed.
        SaveRequest request;
        {
          std::lock_guard<std::mutex> lock(result_mutex_);
          request.valid = has_result_;
          if (request.valid) {
            request.transform = T_result_;
          }
        }
        std::lock_guard<std::mutex> lock(command_mutex_);
        save_requests_.push_back(request);
        return;
      }
      std::lock_guard<std::mutex> lock(command_mutex_);
      command_msgs_.push_back(*msg);
      if (command_msgs_.size() > 1) {
        command_msgs_.pop_front();
      }
    };
  cmd_sub_ = node_->create_subscription<ssct_interfaces::msg::CalibrationCommand>(
    command_topic, 10, command_msg_callback);
  result_sub_ = node_->create_subscription<ssct_interfaces::msg::CalibrationParams>(
    topic_name_prefix + "/result_params", 10,
    [this](const ssct_interfaces::msg::CalibrationParams::SharedPtr msg) {
      on_result_params(msg);
    });
  status_pub_ = node_->create_publisher<ssct_interfaces::msg::CalibrationStatus>(
    topic_name_prefix + "/status", 100);
  rclcpp::QoS params_qos(rclcpp::KeepLast(1));
  params_qos.transient_local();
  params_qos.reliable();
  params_pub_ = node_->create_publisher<ssct_interfaces::msg::CalibrationParams>(
    topic_name_prefix + "/initial_params", params_qos);
  image_pub_ = std::make_shared<ssct_common::ImagePublisher>(
    node_, image_topic_, 100, camera_frame_id_);
  cloud_pub_ = std::make_shared<ssct_common::CloudPublisher>(
    node_, cloud_topic_, 100, lidar_frame_id_);
  publish_initial_params();
  // initialize status
  status_msg_.frame_id = camera_frame_id_;
  status_msg_.sensor_topic = image_topic_;
  status_msg_.child_frame_id = lidar_frame_id_;
  status_msg_.child_sensor_topic = cloud_topic_;
  status_msg_.calibration_type = "ssct_lidar_cam_manual_calib";
  status_msg_.command_topic = command_topic;
  state_ = ssct_interfaces::msg::CalibrationStatus::READY;
  update_status_msg(state_, "ready to start.", 0);
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

void CalibrationNode::publish_initial_params()
{
  const auto & target = image_undistorter_->target_param();
  ssct_interfaces::msg::CalibrationParams msg;
  msg.timestamp = node_->get_clock()->now();
  ssct_interfaces::msg::CameraIntrinsicParam camera;
  camera.frame_id = target.frame_id;
  camera.height = target.height;
  camera.width = target.width;
  camera.type = target.type;
  camera.intrinsics.reserve(target.intrinsics.size());
  for (auto value : target.intrinsics) {
    camera.intrinsics.push_back(static_cast<float>(value));
  }
  camera.distortion_coeffs.reserve(target.distortion_coeffs.size());
  for (auto value : target.distortion_coeffs) {
    camera.distortion_coeffs.push_back(static_cast<float>(value));
  }
  msg.camera_intrinsic_params.push_back(camera);
  msg.extrinsic_params.push_back(
    make_extrinsic_param(camera_frame_id_, lidar_frame_id_, T_camera_lidar_));
  params_pub_->publish(msg);
}

void CalibrationNode::on_result_params(
  const ssct_interfaces::msg::CalibrationParams::SharedPtr msg)
{
  for (const auto & extrinsic : msg->extrinsic_params) {
    if (extrinsic.frame_id != camera_frame_id_ || extrinsic.child_frame_id != lidar_frame_id_) {
      continue;
    }
    Eigen::Matrix4d T;
    if (!parse_extrinsic_param(extrinsic, T)) {
      RCLCPP_WARN(node_->get_logger(), "ignore invalid calibration result.");
      return;
    }
    {
      std::lock_guard<std::mutex> lock(result_mutex_);
      T_result_ = T;
      has_result_ = true;
    }
    RCLCPP_INFO(node_->get_logger(), "received calibration result.");
    return;
  }
  RCLCPP_WARN(
    node_->get_logger(), "calibration result has no extrinsic from %s to %s.",
    lidar_frame_id_.c_str(), camera_frame_id_.c_str());
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
    image = image_buffer_.rbegin()->second.image;
  } else if (cur == image_buffer_.begin()) {
    // the first
    image_timestamp = cur->first;
    image = cur->second.image;
  } else {
    // choose between prev and cur
    auto prev = std::prev(cur);
    if (fabs(prev->first - time) < fabs(cur->first - time)) {
      image_timestamp = prev->first;
      image = prev->second.image;
    } else {
      image_timestamp = cur->first;
      image = cur->second.image;
    }
  }
  return true;
}

bool CalibrationNode::read_data(SensorData & data)
{
  std::deque<ssct_common::CloudSubscriber<pcl::PointXYZI>::MsgData> pointclouds;
  pointcloud_sub_->read(pointclouds);
  for (auto & pointcloud : pointclouds) {
    pointcloud_buffer_.push_back(pointcloud);
  }
  while (pointcloud_buffer_.size() > max_pointcloud_buffer_size_) {
    pointcloud_buffer_.pop_front();
  }
  std::map<double, ssct_common::ImageData> images;
  image_sub_->read(images);
  for (auto & [timestamp, image] : images) {
    image_buffer_[timestamp + image_time_offset_] = image;
  }
  while (image_buffer_.size() > max_image_buffer_size_) {
    image_buffer_.erase(image_buffer_.begin());
  }
  for (auto it = pointcloud_buffer_.begin(); it != pointcloud_buffer_.end(); ++it) {
    double image_timestamp;
    cv::Mat image;
    if (!get_nearest_image(it->time, image_timestamp, image)) {
      continue;
    }
    if (std::fabs(image_timestamp - it->time) > max_time_diff_) {
      continue;
    }
    data.time = it->time;
    data.pointcloud = it->pointcloud;
    data.image = image;
    pointcloud_buffer_.erase(it);
    auto image_it = image_buffer_.find(image_timestamp);
    if (image_it != image_buffer_.end()) {
      image_buffer_.erase(image_buffer_.begin(), std::next(image_it));
    }
    return true;
  }
  return false;
}

void CalibrationNode::clear_data()
{
  pointcloud_sub_->clear();
  image_sub_->clear();
  pointcloud_buffer_.clear();
  image_buffer_.clear();
}

void CalibrationNode::process_command(const ssct_interfaces::msg::CalibrationCommand & msg)
{
  if (msg.command == ssct_interfaces::msg::CalibrationCommand::RESET) {
    state_ = ssct_interfaces::msg::CalibrationStatus::READY;
    {
      std::lock_guard<std::mutex> lock(result_mutex_);
      has_result_ = false;
    }
    data_published_ = false;
    clear_data();
    update_status_msg(state_, "ready to start.", 0);
  } else if (msg.command == ssct_interfaces::msg::CalibrationCommand::START) {
    state_ = ssct_interfaces::msg::CalibrationStatus::CALIBRATING;
    {
      std::lock_guard<std::mutex> lock(result_mutex_);
      has_result_ = false;
    }
    data_published_ = false;
    clear_data();
    capture_start_time_ = std::chrono::steady_clock::now();
    update_status_msg(state_, "waiting for synchronized image and pointcloud.", 1);
  } else {
    RCLCPP_WARN(node_->get_logger(), "unsupported calibration command: %d", msg.command);
  }
}

void CalibrationNode::process_save_requests()
{
  std::deque<SaveRequest> requests;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    requests.swap(save_requests_);
  }
  for (const auto & request : requests) {
    if (!request.valid) {
      update_status_msg(
        state_, "failed to save result, no calibration result received before save command.", 99);
      RCLCPP_WARN(node_->get_logger(), "failed to save, no calibration result received.");
      continue;
    }
    save_result(request.transform);
  }
}

void CalibrationNode::update_status_msg(uint8_t state, const std::string & info, uint8_t progress)
{
  std::lock_guard<std::mutex> lock(status_mutex_);
  status_msg_.timestamp = node_->get_clock()->now();
  status_msg_.state = state;
  status_msg_.info = info;
  status_msg_.progress = progress;
}

void CalibrationNode::save_result(const Eigen::Matrix4d & T_camera_lidar)
{
  if (output_calibration_file_ == "") {
    RCLCPP_FATAL(node_->get_logger(), "failed to save result, no output_calibration_file!");
    update_status_msg(state_, "failed to save result, no output_calibration_file.", 99);
    return;
  }
  auto calib_param_manager = std::make_shared<ssct_common::CalibParamManager>();
  if (std::filesystem::exists(output_calibration_file_)) {
    if (!calib_param_manager->load(output_calibration_file_)) {
      RCLCPP_FATAL(
        node_->get_logger(), "failed to load existed calibration data, %s",
        calib_param_manager->error_message().c_str());
      update_status_msg(state_, "failed to load existed calibration data.", 99);
      return;
    }
  }
  calib_param_manager->add_extrinsic_param(camera_frame_id_, lidar_frame_id_, T_camera_lidar);
  if (calib_param_manager->save(output_calibration_file_)) {
    state_ = ssct_interfaces::msg::CalibrationStatus::SUCCESSED;
    RCLCPP_INFO(
      node_->get_logger(), "successed to save result: %s", output_calibration_file_.c_str());
    update_status_msg(
      state_, "successed to save result: " + output_calibration_file_, 100);
  } else {
    RCLCPP_WARN(
      node_->get_logger(), "failed to save result: %s", output_calibration_file_.c_str());
    update_status_msg(
      state_, "failed to save result: " + output_calibration_file_, 99);
  }
}

bool CalibrationNode::run()
{
  // process the latest command
  {
    std::deque<ssct_interfaces::msg::CalibrationCommand> commands;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      commands.swap(command_msgs_);
    }
    for (const auto & msg : commands) {
      process_command(msg);
    }
  }
  process_save_requests();
  // calibration flow
  if (state_ != ssct_interfaces::msg::CalibrationStatus::CALIBRATING || data_published_) {
    return false;
  }
  SensorData data;
  if (read_data(data)) {
    cv::Mat undistorted;
    if (!image_undistorter_->process(data.image, undistorted)) {
      state_ = ssct_interfaces::msg::CalibrationStatus::FAILED;
      update_status_msg(
        state_, "failed to undistort image, " + image_undistorter_->error_message(), 100);
      return true;
    }
    image_pub_->publish(undistorted, data.time);
    cloud_pub_->publish(*data.pointcloud, data.time);
    data_published_ = true;
    update_status_msg(state_, "finished to publish synchronized data, waiting for save.", 99);
    return true;
  }
  const double elapsed =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - capture_start_time_).count();
  if (elapsed > capture_timeout_) {
    state_ = ssct_interfaces::msg::CalibrationStatus::FAILED;
    update_status_msg(
      state_, "no synchronized image and pointcloud received before timeout.", 100);
    return true;
  }
  return false;
}

}  // namespace ssct_lidar_cam_calib
