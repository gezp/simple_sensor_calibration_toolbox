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

#include <string>

#include "rclcpp/time.hpp"
#include "ssct_common/calib_param/camera_intrinsic_param.hpp"
#include "ssct_common/calib_param/extrinsic_param.hpp"
#include "ssct_common/calib_param_manager.hpp"
#include "ssct_interfaces/msg/calibration_params.hpp"
#include "ssct_interfaces/msg/camera_intrinsic_param.hpp"
#include "ssct_interfaces/msg/extrinsic_param.hpp"

namespace ssct_common
{

ssct_interfaces::msg::ExtrinsicParam to_ros_msg(const ExtrinsicParam & param);
bool from_ros_msg(const ssct_interfaces::msg::ExtrinsicParam & msg, ExtrinsicParam & param);

ssct_interfaces::msg::CameraIntrinsicParam to_ros_msg(const CameraIntrinsicParam & param);
bool from_ros_msg(
  const ssct_interfaces::msg::CameraIntrinsicParam & msg,
  CameraIntrinsicParam & param);

ssct_interfaces::msg::CalibrationParams to_ros_msg(
  const CalibParamManager & params, const rclcpp::Time & stamp);
bool from_ros_msg(
  const ssct_interfaces::msg::CalibrationParams & msg,
  CalibParamManager & params);

}  // namespace ssct_common
