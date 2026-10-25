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

#include "ssct_common/calib_param_msg_utils.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <limits>

#include "ssct_common/msg_utils.hpp"

namespace ssct_common
{

ssct_interfaces::msg::ExtrinsicParam to_ros_msg(const ExtrinsicParam & param)
{
  ssct_interfaces::msg::ExtrinsicParam msg;
  msg.frame_id = param.frame_id;
  msg.child_frame_id = param.child_frame_id;
  msg.transform = to_transform_msg(param.transform);
  return msg;
}

bool from_ros_msg(const ssct_interfaces::msg::ExtrinsicParam & msg, ExtrinsicParam & param)
{
  Eigen::Quaterniond q(
    msg.transform.rotation.w, msg.transform.rotation.x, msg.transform.rotation.y,
    msg.transform.rotation.z);
  if (!q.coeffs().allFinite() || q.norm() < 1e-6) {
    return false;
  }
  q.normalize();
  Eigen::Matrix3d rotation = q.toRotationMatrix();
  if (rotation.determinant() < 0.0) {
    return false;
  }
  param.frame_id = msg.frame_id;
  param.child_frame_id = msg.child_frame_id;
  param.transform = Eigen::Matrix4d::Identity();
  param.transform.block<3, 3>(0, 0) = rotation;
  param.transform(0, 3) = msg.transform.translation.x;
  param.transform(1, 3) = msg.transform.translation.y;
  param.transform(2, 3) = msg.transform.translation.z;
  return true;
}

ssct_interfaces::msg::CameraIntrinsicParam to_ros_msg(const CameraIntrinsicParam & param)
{
  ssct_interfaces::msg::CameraIntrinsicParam msg;
  msg.frame_id = param.frame_id;
  msg.height = static_cast<uint32_t>(std::max(0, param.height));
  msg.width = static_cast<uint32_t>(std::max(0, param.width));
  msg.type = param.type;
  msg.intrinsics.reserve(param.intrinsics.size());
  for (auto value : param.intrinsics) {
    msg.intrinsics.push_back(static_cast<float>(value));
  }
  msg.distortion_coeffs.reserve(param.distortion_coeffs.size());
  for (auto value : param.distortion_coeffs) {
    msg.distortion_coeffs.push_back(static_cast<float>(value));
  }
  return msg;
}

bool from_ros_msg(
  const ssct_interfaces::msg::CameraIntrinsicParam & msg,
  CameraIntrinsicParam & param)
{
  if (
    msg.frame_id.empty() || msg.width == 0 || msg.height == 0 || msg.intrinsics.empty() ||
    msg.width > static_cast<uint32_t>(std::numeric_limits<int>::max()) ||
    msg.height > static_cast<uint32_t>(std::numeric_limits<int>::max()))
  {
    return false;
  }
  param.frame_id = msg.frame_id;
  param.height = static_cast<int>(msg.height);
  param.width = static_cast<int>(msg.width);
  param.type = msg.type;
  param.intrinsics.assign(msg.intrinsics.begin(), msg.intrinsics.end());
  param.distortion_coeffs.assign(msg.distortion_coeffs.begin(), msg.distortion_coeffs.end());
  return true;
}

ssct_interfaces::msg::CalibrationParams to_ros_msg(
  const CalibParamManager & params, const rclcpp::Time & stamp)
{
  ssct_interfaces::msg::CalibrationParams msg;
  msg.timestamp = stamp;
  for (const auto & item : params.camera_intrinsic_params()) {
    msg.camera_intrinsic_params.push_back(to_ros_msg(item.second));
  }
  for (const auto & item : params.extrinsic_params()) {
    msg.extrinsic_params.push_back(to_ros_msg(item.second));
  }
  return msg;
}

bool from_ros_msg(
  const ssct_interfaces::msg::CalibrationParams & msg,
  CalibParamManager & params)
{
  CalibParamManager parsed;
  for (const auto & msg_param : msg.camera_intrinsic_params) {
    CameraIntrinsicParam param;
    if (!from_ros_msg(msg_param, param)) {
      return false;
    }
    parsed.add_camera_intrinsic_param(param);
  }
  for (const auto & msg_param : msg.extrinsic_params) {
    ExtrinsicParam param;
    if (!from_ros_msg(msg_param, param)) {
      return false;
    }
    parsed.add_extrinsic_param(param);
  }
  for (const auto & item : parsed.camera_intrinsic_params()) {
    params.add_camera_intrinsic_param(item.second);
  }
  for (const auto & item : parsed.extrinsic_params()) {
    params.add_extrinsic_param(item.second);
  }
  return true;
}

}  // namespace ssct_common
