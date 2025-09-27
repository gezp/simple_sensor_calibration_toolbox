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

#pragma once

#include <Eigen/Geometry>

#include <string>
#include <vector>
#include <map>
#include <utility>

#include "ssct_common/calib_param/camera_intrinsic_param.hpp"
#include "ssct_common/calib_param/imu_intrinsic_param.hpp"
#include "ssct_common/calib_param/extrinsic_param.hpp"

namespace ssct_common
{

class CalibParamManager
{
public:
  CalibParamManager() = default;
  ~CalibParamManager() = default;
  // for camera intrinsic params
  bool add_camera_intrinsic_param(const CameraIntrinsicParam & param);
  bool get_camera_intrinsic_param(const std::string & frame_id, CameraIntrinsicParam & param);
  void remove_camera_intrinsic_param(const std::string & frame_id);
  // for imu intrinsic params
  bool add_imu_intrinsic_param(const ImuIntrinsicParam & param);
  bool get_imu_intrinsic_param(const std::string & frame_id, ImuIntrinsicParam & param);
  void remove_imu_intrinsic_param(const std::string & frame_id);
  // for extrinsic params
  bool add_extrinsic_param(const ExtrinsicParam & param);
  bool add_extrinsic_param(
    const std::string & frame_id, const std::string & child_frame_id,
    const Eigen::Matrix4d & transform);
  bool get_extrinsic_param(
    const std::string & frame_id, const std::string & child_frame_id, ExtrinsicParam & param);
  void remove_extrinsic_param(const std::string & frame_id, const std::string & child_frame_id);
  // save & load
  bool save(const std::string & file);
  bool load(const std::string & file);
  //
  std::string error_message();

private:
  std::map<std::string, CameraIntrinsicParam> camera_intrinsic_params_;
  std::map<std::string, ImuIntrinsicParam> imu_intrinsic_params_;
  std::map<std::string, ExtrinsicParam> extrinsic_params_;
  std::string error_message_;
};

}  // namespace ssct_common
