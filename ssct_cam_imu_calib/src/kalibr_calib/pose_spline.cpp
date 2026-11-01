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

#include "ssct_cam_imu_calib/kalibr_calib/pose_spline.hpp"

#include <algorithm>
#include <cmath>

#include <sophus/so3.hpp>

#include "ssct_cam_imu_calib/math_utils.hpp"

namespace ssct_cam_imu_calib
{

PoseSpline::PoseSpline(
  double start_time, double step, int num_segments,
  const Eigen::Matrix3d & reference_rotation)
{
  start_time_ = start_time;
  step_ = step;
  num_segments_ = num_segments;
  reference_rotation_ = reference_rotation;
}

int PoseSpline::segment_index(double time) const
{
  if (!valid()) {
    return 0;
  }
  if (time <= start_time_) {
    return 0;
  }
  const double x = (time - start_time_) / step_;
  if (x >= static_cast<double>(num_segments_)) {
    return num_segments_ - 1;
  }
  return std::max(0, std::min(num_segments_ - 1, static_cast<int>(std::floor(x))));
}

void PoseSpline::basis(double u, Eigen::Matrix<double, 4, 1> & values)
{
  const double v = 1.0 - u;
  values[0] = v * v * v / 6.0;
  values[1] = (3.0 * u * u * u - 6.0 * u * u + 4.0) / 6.0;
  values[2] = (-3.0 * u * u * u + 3.0 * u * u + 3.0 * u + 1.0) / 6.0;
  values[3] = u * u * u / 6.0;
}

void PoseSpline::basis_derivative(double u, Eigen::Matrix<double, 4, 1> & values)
{
  values[0] = -0.5 * (1.0 - u) * (1.0 - u);
  values[1] = 1.5 * u * u - 2.0 * u;
  values[2] = -1.5 * u * u + u + 0.5;
  values[3] = 0.5 * u * u;
}

void PoseSpline::basis_second_derivative(
  double u, Eigen::Matrix<double, 4, 1> & values)
{
  values[0] = 1.0 - u;
  values[1] = 3.0 * u - 2.0;
  values[2] = -3.0 * u + 1.0;
  values[3] = u;
}

bool PoseSpline::evaluate(
  const double * const * controls, double time, PoseSample & sample) const
{
  if (!valid() || controls == nullptr) {
    return false;
  }
  const int segment = segment_index(time);
  const double x = (time - start_time_) / step_ - static_cast<double>(segment);
  const double u = std::max(0.0, std::min(1.0, x));

  Eigen::Matrix<double, 4, 1> values;
  Eigen::Matrix<double, 4, 1> derivatives;
  Eigen::Matrix<double, 4, 1> second_derivatives;
  basis(u, values);
  basis_derivative(u, derivatives);
  basis_second_derivative(u, second_derivatives);

  Eigen::Vector3d rotation = Eigen::Vector3d::Zero();
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d rotation_rate = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
  for (int i = 0; i < 4; i++) {
    const double * control = controls[i];
    rotation += values[i] * Eigen::Vector3d(control[0], control[1], control[2]);
    position += values[i] * Eigen::Vector3d(control[3], control[4], control[5]);
    rotation_rate += derivatives[i] / step_ *
      Eigen::Vector3d(control[0], control[1], control[2]);
    velocity += derivatives[i] / step_ *
      Eigen::Vector3d(control[3], control[4], control[5]);
    acceleration += second_derivatives[i] / (step_ * step_) *
      Eigen::Vector3d(control[3], control[4], control[5]);
  }

  sample.T_world_body.setIdentity();
  sample.T_world_body.block<3, 3>(0, 0) =
    reference_rotation_ * Sophus::SO3d::exp(rotation).matrix();
  sample.T_world_body.block<3, 1>(0, 3) = position;
  sample.velocity = velocity;
  sample.acceleration = acceleration;
  sample.angular_velocity = Sophus::SO3d::leftJacobian(-rotation) * rotation_rate;
  return true;
}

bool PoseSpline::evaluate(
  const std::vector<std::array<double, kCoefficients>> & controls, double time,
  PoseSample & sample) const
{
  if (!valid() || controls.size() < static_cast<size_t>(num_controls())) {
    return false;
  }
  const int segment = segment_index(time);
  const double * segment_controls[4] = {
    controls[segment].data(), controls[segment + 1].data(),
    controls[segment + 2].data(), controls[segment + 3].data()};
  return evaluate(segment_controls, time, sample);
}

}  // namespace ssct_cam_imu_calib
