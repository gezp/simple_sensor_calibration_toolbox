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

#include <Eigen/Geometry>

#include <array>
#include <vector>

namespace ssct_cam_imu_calib
{

// Continuous-time pose spline used by the Kalibr-style estimator.
//
// The spline is a uniform cubic B-spline in the Lie algebra of the first
// control pose. A control point stores [rotation increment, translation]:
//   R(t) = R_reference * exp(sum_i B_i(t) * rotation_i)
//   p(t) = sum_i B_i(t) * translation_i
// Rotation increments are expressed in the reference frame. The basis is
// uniform and the domain is [start_time, start_time + num_segments * step].
struct PoseSample
{
  Eigen::Matrix4d T_world_body{Eigen::Matrix4d::Identity()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};
};

class PoseSpline
{
public:
  static constexpr int kOrder = 4;
  static constexpr int kCoefficients = 6;

  PoseSpline() = default;
  PoseSpline(
    double start_time, double step, int num_segments,
    const Eigen::Matrix3d & reference_rotation);

  bool valid() const {return num_segments_ > 0 && step_ > 0.0;}
  double start_time() const {return start_time_;}
  double end_time() const {return start_time_ + num_segments_ * step_;}
  double step() const {return step_;}
  int num_segments() const {return num_segments_;}
  int num_controls() const {return num_segments_ + kOrder - 1;}
  const Eigen::Matrix3d & reference_rotation() const {return reference_rotation_;}

  // Returns the segment containing `time`, clamped to the spline domain.
  int segment_index(double time) const;

  // Evaluates the spline from the four control blocks that start at the
  // segment containing `time`.
  bool evaluate(const double * const * controls, double time, PoseSample & sample) const;

  // Evaluates the spline from all control blocks stored consecutively.
  bool evaluate(
    const std::vector<std::array<double, kCoefficients>> & controls, double time,
    PoseSample & sample) const;

  static void basis(double u, Eigen::Matrix<double, 4, 1> & values);
  static void basis_derivative(double u, Eigen::Matrix<double, 4, 1> & values);
  static void basis_second_derivative(double u, Eigen::Matrix<double, 4, 1> & values);

private:
  double start_time_{0.0};
  double step_{0.1};
  int num_segments_{0};
  Eigen::Matrix3d reference_rotation_{Eigen::Matrix3d::Identity()};
};

}  // namespace ssct_cam_imu_calib
