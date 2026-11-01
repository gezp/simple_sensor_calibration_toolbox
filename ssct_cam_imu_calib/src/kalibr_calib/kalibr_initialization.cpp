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

#include "ssct_cam_imu_calib/kalibr_calib/kalibr_initialization.hpp"

#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <limits>

#include <sophus/so3.hpp>

#include "ssct_cam_imu_calib/kalibr_calib/pose_spline.hpp"
#include "ssct_cam_imu_calib/math_utils.hpp"

namespace ssct_cam_imu_calib
{
namespace
{

constexpr double kEpsilon = 1e-9;

struct AngularVelocityPair
{
  double start_time;
  double end_time;
  Eigen::Vector3d omega_camera;
};

std::vector<AngularVelocityPair> angular_velocity_pairs(
  const std::vector<BoardObservation> & observations)
{
  std::vector<AngularVelocityPair> pairs;
  for (size_t i = 0; i + 1 < observations.size(); i++) {
    const double dt = observations[i + 1].time - observations[i].time;
    if (dt < 0.01 || dt > 0.5) {
      continue;
    }
    // The board is static, so the relative rotation of the board between two
    // frames is the inverse camera rotation, expressed in the camera frame.
    const Eigen::Matrix3d relative_rotation =
      observations[i + 1].T_camera_board.block<3, 3>(0, 0) *
      observations[i].T_camera_board.block<3, 3>(0, 0).transpose();
    const Eigen::Vector3d omega_camera =
      -Sophus::SO3d(relative_rotation).log() / dt;
    if (!omega_camera.allFinite()) {
      continue;
    }
    pairs.push_back({observations[i].time, observations[i + 1].time, omega_camera});
  }
  return pairs;
}

struct PoseAnchor
{
  double time;
  Eigen::Matrix4d T_world_body;
};

// Interpolates the vision anchors, extrapolating linearly with the first/last
// anchor pair outside the observed span. No IMU integration is involved, so the
// initialization does not depend on how the dynamic segment starts.
bool sample_anchor(
  const std::vector<PoseAnchor> & anchors, double time, Eigen::Matrix4d & pose)
{
  if (anchors.empty()) {
    return false;
  }
  size_t right = 1;
  if (time <= anchors.front().time) {
    if (anchors.size() < 2) {
      pose = anchors.front().T_world_body;
      return true;
    }
  } else if (time >= anchors.back().time) {
    if (anchors.size() < 2) {
      pose = anchors.back().T_world_body;
      return true;
    }
    right = anchors.size() - 1;
  } else {
    right = static_cast<size_t>(std::lower_bound(
      anchors.begin(), anchors.end(), time,
        [](const PoseAnchor & anchor, double value) {return anchor.time < value;}) -
      anchors.begin());
  }
  const PoseAnchor & left_anchor = anchors[right - 1];
  const PoseAnchor & right_anchor = anchors[right];
  const double dt = right_anchor.time - left_anchor.time;
  const double alpha = dt > kEpsilon ? (time - left_anchor.time) / dt : 0.0;
  pose = interpolate_pose(left_anchor.T_world_body, right_anchor.T_world_body, alpha);
  return pose.allFinite();
}

}  // namespace

double estimate_time_offset(
  const std::vector<BoardObservation> & observations,
  const std::vector<ssct_common::ImuData> & imu, double range, double step)
{
  const std::vector<AngularVelocityPair> pairs =
    angular_velocity_pairs(observations);
  if (pairs.size() < 3 || range <= 0.0 || step <= 0.0) {
    return 0.0;
  }
  // Only the angular speed magnitude is compared: it is invariant to the
  // unknown camera-IMU rotation and barely affected by the (small) gyro bias,
  // so the time offset can be estimated before the rotation.
  const int steps = static_cast<int>(std::round(2.0 * range / step));
  double best_error = std::numeric_limits<double>::infinity();
  double best_offset = 0.0;
  for (int k = 0; k <= steps; k++) {
    const double offset = -range + k * step;
    double total_error = 0.0;
    size_t count = 0;
    for (const auto & pair : pairs) {
      Eigen::Vector3d gyro_sum = Eigen::Vector3d::Zero();
      size_t gyro_count = 0;
      // offset = t_imu - t_camera, so the IMU window is camera time + offset.
      for (const auto & sample : imu) {
        if (sample.time < pair.start_time + offset) {
          continue;
        }
        if (sample.time > pair.end_time + offset) {
          break;
        }
        gyro_sum += sample.angular_velocity;
        gyro_count++;
      }
      if (gyro_count == 0) {
        continue;
      }
      const double gyro_speed =
        (gyro_sum / static_cast<double>(gyro_count)).norm();
      const double camera_speed = pair.omega_camera.norm();
      total_error += (gyro_speed - camera_speed) * (gyro_speed - camera_speed);
      count++;
    }
    if (count < 3) {
      continue;
    }
    const double error = total_error / static_cast<double>(count);
    if (error < best_error) {
      best_error = error;
      best_offset = offset;
    }
  }
  if (!std::isfinite(best_error) || std::abs(best_offset) >= range - 0.5 * step) {
    // The minimum lies on the search bound, so the search does not contain the
    // true offset; fall back to zero instead of starting the optimizer on the
    // bound.
    return 0.0;
  }
  return best_offset;
}

bool estimate_orientation_prior(
  const std::vector<BoardObservation> & observations,
  const std::vector<ssct_common::ImuData> & imu, double time_offset,
  const Eigen::Matrix3d & initial_R_body_camera,
  Eigen::Matrix3d & R_body_camera, Eigen::Vector3d & gyro_bias)
{
  const std::vector<AngularVelocityPair> pairs =
    angular_velocity_pairs(observations);
  std::vector<Eigen::Vector3d> omega_camera;
  std::vector<Eigen::Vector3d> omega_body;
  for (const auto & pair : pairs) {
    Eigen::Vector3d gyro_sum = Eigen::Vector3d::Zero();
    size_t gyro_count = 0;
    for (const auto & sample : imu) {
      if (sample.time < pair.start_time + time_offset) {
        continue;
      }
      if (sample.time > pair.end_time + time_offset) {
        break;
      }
      gyro_sum += sample.angular_velocity;
      gyro_count++;
    }
    if (gyro_count == 0) {
      continue;
    }
    omega_camera.push_back(pair.omega_camera);
    omega_body.push_back(gyro_sum / static_cast<double>(gyro_count));
  }
  if (omega_camera.size() < 3) {
    return false;
  }

  // Alternate between the gyro bias and the Wahba rotation estimate; this is
  // the closed-form equivalent of Kalibr's joint gyro/rotation prior.
  Eigen::Matrix3d R = initial_R_body_camera;
  Eigen::Vector3d bias = Eigen::Vector3d::Zero();
  for (int iteration = 0; iteration < 20; iteration++) {
    bias.setZero();
    for (size_t i = 0; i < omega_body.size(); i++) {
      bias += omega_body[i] - R * omega_camera[i];
    }
    bias /= static_cast<double>(omega_body.size());
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (size_t i = 0; i < omega_body.size(); i++) {
      covariance += (omega_body[i] - bias) * omega_camera[i].transpose();
    }
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Eigen::Matrix3d U = svd.matrixU();
    const Eigen::Matrix3d V = svd.matrixV();
    Eigen::Matrix3d D = Eigen::Matrix3d::Identity();
    D(2, 2) = (U * V.transpose()).determinant() < 0.0 ? -1.0 : 1.0;
    R = U * D * V.transpose();
  }
  if (!R.allFinite() || !bias.allFinite()) {
    return false;
  }
  R_body_camera = R;
  gyro_bias = bias;
  return true;
}

namespace
{
// Gravity in the world frame from the mean specific force expressed in the world
// frame. The mean world acceleration is small when the dynamic segment starts
// and ends with a similar velocity, so the mean of R_wb * f_b is approximately
// -g. Only the direction is estimated; the magnitude is fixed by the caller,
// as Kalibr does by default.
Eigen::Vector3d estimate_gravity_world(
  const std::vector<ssct_common::ImuData> & imu,
  const PoseSpline & spline, const std::vector<std::array<double, 6>> & controls,
  double gravity_magnitude)
{
  Eigen::Vector3d specific_force = Eigen::Vector3d::Zero();
  size_t count = 0;
  for (const auto & sample : imu) {
    PoseSample pose;
    if (!spline.evaluate(controls, sample.time, pose)) {
      continue;
    }
    specific_force += pose.T_world_body.block<3, 3>(0, 0) * sample.linear_acceleration;
    count++;
  }
  if (count == 0 || specific_force.norm() < kEpsilon) {
    return Eigen::Vector3d(0.0, 0.0, -gravity_magnitude);
  }
  return -gravity_magnitude * specific_force.normalized();
}
}  // namespace

bool build_pose_spline(
  const std::vector<BoardObservation> & observations,
  const std::vector<ssct_common::ImuData> & imu,
  const Eigen::Matrix4d & T_camera_imu,
  double time_offset, double max_time_offset, double knot_spacing,
  double gravity_magnitude,
  PoseSpline & spline, std::vector<std::array<double, 6>> & controls,
  Eigen::Vector3d & gravity_world)
{
  std::vector<PoseAnchor> anchors;
  anchors.reserve(observations.size());
  for (const auto & observation : observations) {
    PoseAnchor anchor;
    anchor.time = observation.time + time_offset;
    anchor.T_world_body = observation.T_camera_board.inverse() * T_camera_imu;
    if (!anchor.T_world_body.allFinite()) {
      continue;
    }
    anchors.push_back(anchor);
  }
  if (anchors.size() < 2 || imu.size() < 2) {
    return false;
  }
  std::sort(
    anchors.begin(), anchors.end(),
    [](const PoseAnchor & left, const PoseAnchor & right) {
      return left.time < right.time;
    });

  // The spline only needs to cover the camera observations evaluated over the
  // whole time-offset range; extending it over the whole IMU stream would add
  // segments without camera support.
  double data_start = std::numeric_limits<double>::infinity();
  double data_end = -std::numeric_limits<double>::infinity();
  for (const auto & observation : observations) {
    data_start = std::min(data_start, observation.time);
    data_end = std::max(data_end, observation.time);
  }
  data_start -= max_time_offset;
  data_end += max_time_offset;
  const double span = std::max(1e-3, data_end - data_start);
  const double step = std::max(0.01, std::min(knot_spacing, span));
  const double start = data_start - step;
  const int num_segments = std::max(1, static_cast<int>(std::ceil((data_end - start) / step)));
  const int num_controls = num_segments + PoseSpline::kOrder - 1;
  if (num_controls < PoseSpline::kOrder) {
    return false;
  }

  Eigen::Matrix4d reference_pose;
  if (!sample_anchor(anchors, start, reference_pose)) {
    return false;
  }
  const Eigen::Matrix3d reference_rotation = reference_pose.block<3, 3>(0, 0);
  spline = PoseSpline(start, step, num_segments, reference_rotation);
  controls.assign(num_controls, std::array<double, 6>{});

  for (int i = 0; i < num_controls; i++) {
    const double control_time = start + (static_cast<double>(i) - 1.0) * step;
    Eigen::Matrix4d pose;
    if (!sample_anchor(anchors, control_time, pose)) {
      return false;
    }
    const Eigen::Vector3d rotation =
      Sophus::SO3d(reference_rotation.transpose() * pose.block<3, 3>(0, 0)).log();
    controls[i][0] = rotation.x();
    controls[i][1] = rotation.y();
    controls[i][2] = rotation.z();
    controls[i][3] = pose(0, 3);
    controls[i][4] = pose(1, 3);
    controls[i][5] = pose(2, 3);
  }

  gravity_world = estimate_gravity_world(imu, spline, controls, gravity_magnitude);
  return true;
}

}  // namespace ssct_cam_imu_calib
