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

#include <ceres/ceres.h>

#include <Eigen/Core>

#include <cmath>
#include <vector>

#include "ssct_common/sensor_data/imu_data.hpp"

namespace ssct_imu_intrinsic_calib
{

// Same residual as IMU-TK, with the analytic Jacobian from its
// MultiPosAccAnalyticCostFunction.
class AccelCost : public ceres::SizedCostFunction<1, 9>
{
public:
  AccelCost(double x, double y, double z, double gravity)
  : x_(x), y_(y), z_(z), gravity_(gravity) {}

  bool Evaluate(
    const double * const * parameters, double * residuals,
    double ** jacobians) const override
  {
    const double * params = parameters[0];
    const double dx = x_ - params[6];
    const double dy = y_ - params[7];
    const double dz = z_ - params[8];
    const double c_x = params[3] * dx;
    const double c_y = params[4] * dy;
    const double c_z = params[5] * dz;
    const double a_x = c_x;
    const double a_y = params[0] * c_x + c_y;
    const double a_z = -params[1] * c_x + params[2] * c_y + c_z;
    const double norm = std::sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
    if (!(norm > 0.0)) {
      return false;
    }
    residuals[0] = gravity_ - norm;
    if (jacobians != nullptr && jacobians[0] != nullptr) {
      Eigen::Map<Eigen::Matrix<double, 1, 9>> jacobian(jacobians[0]);
      Eigen::Matrix<double, 1, 3> df_by_da =
        -Eigen::Matrix<double, 1, 3>(a_x, a_y, a_z) / norm;
      Eigen::Matrix<double, 3, 9> da_by_dtheta = Eigen::Matrix<double, 3, 9>::Zero();
      da_by_dtheta(1, 0) = c_x;
      da_by_dtheta(2, 1) = -c_x;
      da_by_dtheta(2, 2) = c_y;
      da_by_dtheta.col(3) << dx, params[0] * dx, -params[1] * dx;
      da_by_dtheta.col(4) << 0.0, dy, params[2] * dy;
      da_by_dtheta.col(5) << 0.0, 0.0, dz;
      da_by_dtheta.col(6) <<
        -params[3], -params[0] * params[3], params[1] * params[3];
      da_by_dtheta.col(7) << 0.0, -params[4], -params[2] * params[4];
      da_by_dtheta(2, 8) = -params[5];
      jacobian = df_by_da * da_by_dtheta;
    }
    return true;
  }

private:
  double x_;
  double y_;
  double z_;
  double gravity_;
};

template<typename T>
void quat_omega_skew(
  const Eigen::Matrix<T, 3, 1> & omega, Eigen::Matrix<T, 4, 4> & skew)
{
  skew << T(0), -omega[0], -omega[1], -omega[2],
    omega[0], T(0), omega[2], -omega[1],
    omega[1], -omega[2], T(0), omega[0],
    omega[2], omega[1], -omega[0], T(0);
}

template<typename T>
void integrate_step(
  const Eigen::Matrix<T, 4, 1> & quat, const Eigen::Matrix<T, 3, 1> & omega_0,
  const Eigen::Matrix<T, 3, 1> & omega_1, double dt,
  Eigen::Matrix<T, 4, 1> & result)
{
  const Eigen::Matrix<T, 3, 1> omega_01 = T(0.5) * (omega_0 + omega_1);
  Eigen::Matrix<T, 4, 4> skew;
  Eigen::Matrix<T, 4, 1> k1;
  Eigen::Matrix<T, 4, 1> k2;
  Eigen::Matrix<T, 4, 1> k3;
  Eigen::Matrix<T, 4, 1> k4;
  quat_omega_skew(omega_0, skew);
  k1 = T(0.5) * skew * quat;
  Eigen::Matrix<T, 4, 1> tmp = quat + T(0.5 * dt) * k1;
  quat_omega_skew(omega_01, skew);
  k2 = T(0.5) * skew * tmp;
  tmp = quat + T(0.5 * dt) * k2;
  k3 = T(0.5) * skew * tmp;
  tmp = quat + T(dt) * k3;
  quat_omega_skew(omega_1, skew);
  k4 = T(0.5) * skew * tmp;
  result = quat + T(dt) *
    (k1 / T(6.0) + k2 / T(3.0) + k3 / T(3.0) + k4 / T(6.0));
  result.normalize();
}

struct GyroCost
{
  GyroCost(
    const std::vector<ssct_common::ImuData> & samples, int start, int end,
    const Eigen::Vector3d & gravity_0, const Eigen::Vector3d & gravity_1,
    const Eigen::Vector3d & bias)
  : samples_(samples), start_(start), end_(end), gravity_0_(gravity_0),
    gravity_1_(gravity_1), bias_(bias) {}

  template<typename T>
  bool operator()(const T * const params, T * residuals) const
  {
    Eigen::Matrix<T, 3, 3> matrix;
    matrix << params[6], -params[0] * params[7], params[1] * params[8],
      params[3] * params[6], params[7], -params[2] * params[8],
      -params[4] * params[6], params[5] * params[7], params[8];
    const Eigen::Matrix<T, 3, 1> bias = bias_.template cast<T>();
    Eigen::Matrix<T, 4, 1> quat(T(1), T(0), T(0), T(0));
    for (int i = start_; i + 1 < end_; i++) {
      const double dt = samples_[i + 1].time - samples_[i].time;
      if (!(dt > 0.0)) {
        continue;
      }
      const Eigen::Matrix<T, 3, 1> omega_0 =
        matrix * (samples_[i].angular_velocity.template cast<T>() - bias);
      const Eigen::Matrix<T, 3, 1> omega_1 =
        matrix * (samples_[i + 1].angular_velocity.template cast<T>() - bias);
      integrate_step(quat, omega_0, omega_1, dt, quat);
    }
    const T w = quat[0];
    const T x = quat[1];
    const T y = quat[2];
    const T z = quat[3];
    Eigen::Matrix<T, 3, 3> rotation;
    rotation << T(1) - T(2) * (y * y + z * z), T(2) * (x * y - z * w),
      T(2) * (x * z + y * w),
      T(2) * (x * y + z * w), T(1) - T(2) * (x * x + z * z),
      T(2) * (y * z - x * w),
      T(2) * (x * z - y * w), T(2) * (y * z + x * w),
      T(1) - T(2) * (x * x + y * y);
    const Eigen::Matrix<T, 3, 1> diff =
      rotation.transpose() * gravity_0_.template cast<T>() -
      gravity_1_.template cast<T>();
    residuals[0] = diff[0];
    residuals[1] = diff[1];
    residuals[2] = diff[2];
    return true;
  }

private:
  const std::vector<ssct_common::ImuData> & samples_;
  int start_;
  int end_;
  Eigen::Vector3d gravity_0_;
  Eigen::Vector3d gravity_1_;
  Eigen::Vector3d bias_;
};

}  // namespace ssct_imu_intrinsic_calib
