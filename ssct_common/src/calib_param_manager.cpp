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

#include <yaml-cpp/yaml.h>

#include <sstream>
#include <fstream>

#include "ssct_common/calib_param_manager.hpp"

namespace ssct_common
{

std::string to_string(const std::vector<double> & data)
{
  std::stringstream ss;
  ss << "[";
  for (size_t i = 0; i < data.size(); i++) {
    ss << data[i];
    if (i + 1 != data.size()) {
      ss << ", ";
    }
  }
  ss << "]";
  return ss.str();
}

std::string to_string(const Eigen::Matrix3d & data)
{
  std::vector<double> vec(data.data(), data.data() + data.size());
  return to_string(vec);
}

std::string to_string(const Eigen::Vector3d & data)
{
  std::vector<double> vec(data.data(), data.data() + data.size());
  return to_string(vec);
}

Eigen::Matrix3d to_Matrix3d(const std::vector<double> & data)
{
  assert(data.size() == 9);
  return Eigen::Map<const Eigen::Matrix3d>(data.data());
}

Eigen::Vector3d to_Vector3d(const std::vector<double> & data)
{
  assert(data.size() == 3);
  return Eigen::Map<const Eigen::Vector3d>(data.data());
}

bool CalibParamManager::add_camera_intrinsic_param(const CameraIntrinsicParam & param)
{
  camera_intrinsic_params_[param.frame_id] = param;
  return true;
}

bool CalibParamManager::get_camera_intrinsic_param(
  const std::string & frame_id, CameraIntrinsicParam & param)
{
  auto it = camera_intrinsic_params_.find(frame_id);
  if (it == camera_intrinsic_params_.end()) {
    return false;
  }
  param = it->second;
  return true;
}

void CalibParamManager::remove_camera_intrinsic_param(const std::string & frame_id)
{
  auto it = camera_intrinsic_params_.find(frame_id);
  if (it != camera_intrinsic_params_.end()) {
    camera_intrinsic_params_.erase(it);
  }
}

bool CalibParamManager::add_imu_intrinsic_param(const ImuIntrinsicParam & param)
{
  imu_intrinsic_params_[param.frame_id] = param;
  return true;
}

bool CalibParamManager::get_imu_intrinsic_param(
  const std::string & frame_id, ImuIntrinsicParam & param)
{
  auto it = imu_intrinsic_params_.find(frame_id);
  if (it == imu_intrinsic_params_.end()) {
    return false;
  }
  param = it->second;
  return true;
}

void CalibParamManager::remove_imu_intrinsic_param(const std::string & frame_id)
{
  auto it = imu_intrinsic_params_.find(frame_id);
  if (it != imu_intrinsic_params_.end()) {
    imu_intrinsic_params_.erase(it);
  }
}

bool CalibParamManager::add_extrinsic_param(const ExtrinsicParam & param)
{
  std::string key = param.frame_id + "_tf_" + param.child_frame_id;
  extrinsic_params_[key] = param;
  return true;
}

bool CalibParamManager::add_extrinsic_param(
  const std::string & frame_id, const std::string & child_frame_id,
  const Eigen::Matrix4d & transform)
{
  ExtrinsicParam param;
  param.frame_id = frame_id;
  param.child_frame_id = child_frame_id;
  param.transform = transform;
  return add_extrinsic_param(param);
}

bool CalibParamManager::get_extrinsic_param(
  const std::string & frame_id, const std::string & child_frame_id, ExtrinsicParam & param)
{
  std::string key = frame_id + "_tf_" + child_frame_id;
  auto it = extrinsic_params_.find(key);
  if (it == extrinsic_params_.end()) {
    return false;
  }
  param = it->second;
  return true;
}

void CalibParamManager::remove_extrinsic_param(
  const std::string & frame_id, const std::string & child_frame_id)
{
  auto key = frame_id + "_tf_" + child_frame_id;
  auto it = extrinsic_params_.find(key);
  if (it != extrinsic_params_.end()) {
    extrinsic_params_.erase(it);
  }
}

bool CalibParamManager::save(const std::string & file)
{
  std::ofstream ofs;
  ofs.open(file);
  if (!ofs) {
    return false;
  }
  ofs.setf(std::ios::fixed, std::ios::floatfield);
  ofs.precision(6);
  if (!camera_intrinsic_params_.empty()) {
    ofs << "cameras:" << std::endl;
    int cnt = 1;
    for (auto & [key, param] : camera_intrinsic_params_) {
      ofs << "    camera" << cnt << ":" << std::endl;
      ofs << "        frame_id: " << param.frame_id << std::endl;
      ofs << "        height: " << param.height << std::endl;
      ofs << "        width: " << param.width << std::endl;
      ofs << "        type: " << param.type << std::endl;
      ofs << "        intrinsics: " << to_string(param.intrinsics) << std::endl;
      ofs << "        distortion_coeffs: " << to_string(param.distortion_coeffs) << std::endl;
      cnt++;
    }
  }
  if (!imu_intrinsic_params_.empty()) {
    ofs << "imus:" << std::endl;
    int cnt = 1;
    for (auto & [key, param] : imu_intrinsic_params_) {
      ofs << "    imu" << cnt << ":" << std::endl;
      ofs << "        frame_id: " << param.frame_id << std::endl;
      ofs << "        accel_matrix: " << to_string(param.accel_matrix) << std::endl;
      ofs << "        accel_offset: " << to_string(param.accel_offset) << std::endl;
      ofs << "        accel_noise_density: " << to_string(param.accel_noise_density) << std::endl;
      ofs << "        accel_random_walk: " << to_string(param.accel_random_walk) << std::endl;
      ofs << "        gyro_matrix: " << to_string(param.gyro_matrix) << std::endl;
      ofs << "        gyro_offset: " << to_string(param.gyro_offset) << std::endl;
      ofs << "        gyro_noise_density: " << to_string(param.gyro_noise_density) << std::endl;
      ofs << "        gyro_random_walk: " << to_string(param.gyro_random_walk) << std::endl;
      cnt++;
    }
  }
  if (!extrinsic_params_.empty()) {
    ofs << "transforms:" << std::endl;
    int cnt = 1;
    for (auto & [key, param] : extrinsic_params_) {
      auto t = param.transform.block<3, 1>(0, 3);
      std::vector<double> translation{t.x(), t.y(), t.z()};
      auto q = Eigen::Quaterniond(param.transform.block<3, 3>(0, 0));
      std::vector<double> rotation{q.x(), q.y(), q.z(), q.w()};
      ofs << "    transform" << cnt << ":" << std::endl;
      ofs << "        frame_id: " << param.frame_id << std::endl;
      ofs << "        child_frame_id: " << param.child_frame_id << std::endl;
      ofs << "        translation: " << to_string(translation) << std::endl;
      ofs << "        rotation: " << to_string(rotation) << std::endl;
      cnt++;
    }
  }
  ofs << std::endl;
  return true;
}

bool CalibParamManager::load(const std::string & file)
{
  camera_intrinsic_params_.clear();
  extrinsic_params_.clear();
  YAML::Node config = YAML::LoadFile(file);
  auto cameras = config["cameras"];
  if (cameras.IsDefined() && cameras.IsMap()) {
    for (auto it = cameras.begin(); it != cameras.end(); ++it) {
      std::string key = "unknown";
      try {
        key = it->first.as<std::string>();
        YAML::Node camera = it->second;
        CameraIntrinsicParam param;
        param.frame_id = camera["frame_id"].as<std::string>();
        param.height = camera["height"].as<int>();
        param.width = camera["width"].as<int>();
        param.intrinsics = camera["intrinsics"].as<std::vector<double>>();
        param.distortion_coeffs = camera["distortion_coeffs"].as<std::vector<double>>();
        add_camera_intrinsic_param(param);
      } catch (std::exception & e) {
        error_message_ = std::string("invalid camera data [") + key + "]";
        return false;
      }
    }
  }
  auto imus = config["imus"];
  if (imus.IsDefined() && imus.IsMap()) {
    for (auto it = cameras.begin(); it != imus.end(); ++it) {
      std::string key = "unknown";
      try {
        key = it->first.as<std::string>();
        YAML::Node camera = it->second;
        ImuIntrinsicParam param;
        param.frame_id = camera["frame_id"].as<std::string>();
        param.accel_matrix = to_Matrix3d(camera["accel_matrix"].as<std::vector<double>>());
        param.accel_offset = to_Vector3d(camera["accel_offset"].as<std::vector<double>>());
        param.accel_noise_density =
          to_Vector3d(camera["accel_noise_density"].as<std::vector<double>>());
        param.accel_random_walk =
          to_Vector3d(camera["accel_random_walk"].as<std::vector<double>>());
        param.gyro_matrix = to_Matrix3d(camera["gyro_matrix"].as<std::vector<double>>());
        param.gyro_offset = to_Vector3d(camera["gyro_offset"].as<std::vector<double>>());
        param.gyro_noise_density =
          to_Vector3d(camera["gyro_noise_density"].as<std::vector<double>>());
        param.gyro_random_walk = to_Vector3d(camera["gyro_random_walk"].as<std::vector<double>>());
        add_imu_intrinsic_param(param);
      } catch (std::exception & e) {
        error_message_ = std::string("invalid imu data [") + key + "]";
        return false;
      }
    }
  }
  auto transforms = config["transforms"];
  if (transforms.IsDefined() && transforms.IsMap()) {
    for (auto it = transforms.begin(); it != transforms.end(); ++it) {
      std::string key = "unknown";
      try {
        key = it->first.as<std::string>();
        YAML::Node transform_node = it->second;
        auto frame_id = transform_node["frame_id"].as<std::string>();
        auto child_frame_id = transform_node["child_frame_id"].as<std::string>();
        auto t = transform_node["translation"].as<std::vector<double>>();
        auto r = transform_node["rotation"].as<std::vector<double>>();
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
        transform.block<3, 1>(0, 3) = Eigen::Vector3d(t[0], t[1], t[2]);
        transform.block<3, 3>(0, 0) = Eigen::Quaterniond(r[3], r[0], r[1], r[2]).toRotationMatrix();
        add_extrinsic_param(frame_id, child_frame_id, transform);
      } catch (std::exception & e) {
        error_message_ = std::string("invalid transform [") + key + "]";
        return false;
      }
    }
  }
  return true;
}

std::string CalibParamManager::error_message()
{
  return error_message_;
}

}  // namespace ssct_common
