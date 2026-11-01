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

#include "ssct_cam_imu_calib/handeye_calib/handeye_optimizer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

#include <Eigen/Dense>
#include <sophus/so3.hpp>

#include "ssct_cam_imu_calib/math_utils.hpp"

namespace ssct_cam_imu_calib
{
namespace
{

struct HandEyeSolution
{
  Eigen::Matrix4d T_body_camera{Eigen::Matrix4d::Identity()};
  double rotation_error{std::numeric_limits<double>::infinity()};
  double translation_error{std::numeric_limits<double>::infinity()};
};

// Solves A * X = X * B for X = T_body_camera. Rotation comes from the relative
// rotation axes (Wahba), translation from the linear system once X is known.
bool solve_hand_eye(
  const std::vector<Eigen::Matrix4d> & body_motions,
  const std::vector<Eigen::Matrix4d> & camera_motions,
  HandEyeSolution & solution)
{
  Eigen::Matrix3d correlation = Eigen::Matrix3d::Zero();
  for (size_t i = 0; i < body_motions.size(); i++) {
    const Eigen::Vector3d body_rotation =
      Sophus::SO3d(body_motions[i].block<3, 3>(0, 0)).log();
    const Eigen::Vector3d camera_rotation =
      Sophus::SO3d(camera_motions[i].block<3, 3>(0, 0)).log();
    correlation += body_rotation * camera_rotation.transpose();
  }
  const Eigen::JacobiSVD<Eigen::Matrix3d> svd(
    correlation, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d correction = Eigen::Matrix3d::Identity();
  if ((svd.matrixU() * svd.matrixV().transpose()).determinant() < 0.0) {
    correction(2, 2) = -1.0;
  }
  const Eigen::Matrix3d rotation =
    svd.matrixU() * correction * svd.matrixV().transpose();
  if (!rotation.allFinite()) {
    return false;
  }

  const size_t count = body_motions.size();
  Eigen::MatrixXd lhs = Eigen::MatrixXd::Zero(3 * count, 3);
  Eigen::VectorXd rhs = Eigen::VectorXd::Zero(3 * count);
  for (size_t i = 0; i < count; i++) {
    lhs.block<3, 3>(3 * i, 0) =
      body_motions[i].block<3, 3>(0, 0) - Eigen::Matrix3d::Identity();
    rhs.segment<3>(3 * i) =
      rotation * camera_motions[i].block<3, 1>(0, 3) -
      body_motions[i].block<3, 1>(0, 3);
  }
  const Eigen::Vector3d translation = lhs.colPivHouseholderQr().solve(rhs);
  if (!translation.allFinite()) {
    return false;
  }

  solution.T_body_camera.setIdentity();
  solution.T_body_camera.block<3, 3>(0, 0) = rotation;
  solution.T_body_camera.block<3, 1>(0, 3) = translation;
  solution.rotation_error = 0.0;
  solution.translation_error = 0.0;
  for (size_t i = 0; i < count; i++) {
    const Eigen::Matrix4d left = body_motions[i] * solution.T_body_camera;
    const Eigen::Matrix4d right = solution.T_body_camera * camera_motions[i];
    solution.rotation_error += rotation_angle(
      left.block<3, 3>(0, 0).transpose() * right.block<3, 3>(0, 0));
    solution.translation_error +=
      (left.block<3, 1>(0, 3) - right.block<3, 1>(0, 3)).norm();
  }
  solution.rotation_error /= static_cast<double>(count);
  solution.translation_error /= static_cast<double>(count);
  return true;
}

}  // namespace

HandEyeOptimizer::HandEyeOptimizer(const YAML::Node & config)
{
  max_score_ = config["max_score"].as<double>();
  max_pairs_ = config["max_pairs"].as<int>();
}

void HandEyeOptimizer::add_data(const std::vector<HandEyePair> & pairs)
{
  pairs_ = pairs;
}

bool HandEyeOptimizer::optimize()
{
  if (pairs_.size() < 2) {
    message_ = "hand-eye needs at least 2 static A->B pairs";
    return false;
  }
  // Keep at most max_pairs_ pairs by taking every stride-th one.
  const size_t stride = max_pairs_ > 0 ?
    std::max<size_t>(1, (pairs_.size() + max_pairs_ - 1) / static_cast<size_t>(max_pairs_)) : 1;
  std::vector<Eigen::Matrix4d> body_motions;
  std::vector<Eigen::Matrix4d> camera_motions;
  double rotation_sum = 0.0;
  double drift_sum = 0.0;
  for (size_t i = 0; i < pairs_.size(); i += stride) {
    body_motions.push_back(pairs_[i].T_imu_a_b);
    camera_motions.push_back(pairs_[i].T_cam_a_b);
    rotation_sum += radians_to_degrees(
      rotation_angle(pairs_[i].T_cam_a_b.block<3, 3>(0, 0)));
    drift_sum += pairs_[i].position_drift;
  }
  if (body_motions.size() < 2) {
    message_ = "hand-eye needs at least 2 usable IMU integrations";
    return false;
  }
  const double mean_rotation_deg =
    rotation_sum / static_cast<double>(body_motions.size());
  HandEyeSolution solution;
  if (!solve_hand_eye(body_motions, camera_motions, solution)) {
    message_ = "failed to solve hand-eye A * X = X * B";
    return false;
  }
  if (solution.rotation_error > max_score_) {
    message_ = "hand-eye rotation error " +
      std::to_string(solution.rotation_error) + " rad is above " +
      std::to_string(max_score_) + " rad";
    return false;
  }
  // X = T_body_camera; output T_camera_body for the camera_tf_imu entry.
  result_.transform = solution.T_body_camera.inverse();
  result_.time_offset = 0.0;
  const Eigen::Quaterniond rotation(result_.transform.block<3, 3>(0, 0));
  std::ostringstream stream;
  stream << "hand-eye: pairs " << body_motions.size()
         << ", mean rotation " << mean_rotation_deg << " deg"
         << ", rotation error " << solution.rotation_error << " rad"
         << ", translation residual " << solution.translation_error << " m"
         << ", mean position drift "
         << drift_sum / static_cast<double>(body_motions.size()) << " m"
         << "\nresult: translation [" << result_.transform(0, 3) << ", "
         << result_.transform(1, 3) << ", " << result_.transform(2, 3) << "]"
         << ", rotation_xyzw [" << rotation.x() << ", " << rotation.y() << ", "
         << rotation.z() << ", " << rotation.w() << "]"
         << ", time_offset " << result_.time_offset << " s";
  message_ = stream.str();
  return true;
}

}  // namespace ssct_cam_imu_calib
