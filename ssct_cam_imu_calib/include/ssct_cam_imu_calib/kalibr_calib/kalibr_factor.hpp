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
#include <Eigen/Geometry>

#include <opencv2/core/types.hpp>
#include <sophus/so3.hpp>

#include "ssct_cam_imu_calib/kalibr_calib/pose_spline.hpp"

namespace ssct_cam_imu_calib
{

// Reprojection residual of one board point.
//
// Parameter blocks: [active control points..., T_camera_imu, time_offset].
// time_offset follows the calibration-file convention t_imu = t_camera + offset,
// so the spline is evaluated at the IMU time image_time + time_offset.
struct ReprojectionFactor
{
  ReprojectionFactor(
    const PoseSpline & spline, int first_control, int control_count, double image_time,
    const Eigen::Vector3d & point, const cv::Point2f & normalized_point,
    double focal_x, double focal_y, double sigma)
  : spline(spline), first_control(first_control), control_count(control_count),
    image_time(image_time), point(point), normalized_point(normalized_point),
    focal_x(focal_x), focal_y(focal_y), sigma(sigma) {}

  bool operator()(const double * const * parameters, double * residual) const
  {
    const double time = image_time + parameters[control_count + 1][0];
    const int segment = spline.segment_index(time);
    const int local_control = segment - first_control;
    PoseSample sample;
    if (local_control < 0 || local_control + 3 >= control_count ||
      !spline.evaluate(parameters + local_control, time, sample))
    {
      residual[0] = 1e3;
      residual[1] = 1e3;
      return true;
    }

    const double * extrinsic = parameters[control_count];
    const Eigen::Matrix3d R_camera_imu =
      Sophus::SO3d::exp(Eigen::Map<const Eigen::Vector3d>(extrinsic)).matrix();
    const Eigen::Vector3d t_camera_imu =
      Eigen::Map<const Eigen::Vector3d>(extrinsic + 3);
    const Eigen::Vector3d point_body =
      sample.T_world_body.block<3, 3>(0, 0).transpose() *
      (point - sample.T_world_body.block<3, 1>(0, 3));
    const Eigen::Vector3d point_camera = R_camera_imu * point_body + t_camera_imu;
    if (point_camera.z() < 1e-6) {
      residual[0] = 1e3;
      residual[1] = 1e3;
      return true;
    }
    residual[0] = focal_x * (point_camera.x() / point_camera.z() -
      normalized_point.x) / sigma;
    residual[1] = focal_y * (point_camera.y() / point_camera.z() -
      normalized_point.y) / sigma;
    return true;
  }

  static ceres::CostFunction * Create(
    const PoseSpline & spline, int first_control, int control_count, double image_time,
    const Eigen::Vector3d & point, const cv::Point2f & normalized_point,
    double focal_x, double focal_y, double sigma)
  {
    auto * cost = new ceres::DynamicNumericDiffCostFunction<ReprojectionFactor>(
      new ReprojectionFactor(
        spline, first_control, control_count, image_time, point,
        normalized_point, focal_x, focal_y, sigma));
    for (int i = 0; i < control_count; i++) {
      cost->AddParameterBlock(6);
    }
    cost->AddParameterBlock(6);
    cost->AddParameterBlock(1);
    cost->SetNumResiduals(2);
    return cost;
  }

  PoseSpline spline;
  int first_control;
  int control_count;
  double image_time;
  Eigen::Vector3d point;
  cv::Point2f normalized_point;
  double focal_x;
  double focal_y;
  double sigma;
};

// Gyro and accelerometer residual of one IMU sample.
//
// Parameter blocks: 4 control points, gyro bias, accel bias, gravity.
struct ImuFactor
{
  ImuFactor(
    const PoseSpline & spline, double time,
    const Eigen::Vector3d & gyro_measurement, const Eigen::Vector3d & accel_measurement,
    double gyro_sigma, double accel_sigma)
  : spline(spline), time(time), gyro_measurement(gyro_measurement),
    accel_measurement(accel_measurement), gyro_sigma(gyro_sigma),
    accel_sigma(accel_sigma) {}

  bool operator()(const double * const * parameters, double * residual) const
  {
    PoseSample sample;
    if (!spline.evaluate(parameters, time, sample)) {
      for (int i = 0; i < 6; i++) {
        residual[i] = 1e3;
      }
      return true;
    }
    const Eigen::Vector3d gyro_bias(
      parameters[4][0], parameters[4][1], parameters[4][2]);
    const Eigen::Vector3d accel_bias(
      parameters[5][0], parameters[5][1], parameters[5][2]);
    const Eigen::Vector3d gravity(
      parameters[6][0], parameters[6][1], parameters[6][2]);
    const Eigen::Vector3d gyro_error =
      sample.angular_velocity - (gyro_measurement - gyro_bias);
    const Eigen::Vector3d specific_force =
      sample.T_world_body.block<3, 3>(0, 0).transpose() *
      (sample.acceleration - gravity);
    const Eigen::Vector3d accel_error =
      specific_force - (accel_measurement - accel_bias);
    for (int i = 0; i < 3; i++) {
      residual[i] = gyro_error[i] / gyro_sigma;
      residual[3 + i] = accel_error[i] / accel_sigma;
    }
    return true;
  }

  static ceres::CostFunction * Create(
    const PoseSpline & spline, double time,
    const Eigen::Vector3d & gyro_measurement, const Eigen::Vector3d & accel_measurement,
    double gyro_sigma, double accel_sigma)
  {
    auto * cost = new ceres::DynamicNumericDiffCostFunction<ImuFactor>(
      new ImuFactor(
        spline, time, gyro_measurement, accel_measurement, gyro_sigma, accel_sigma));
    for (int i = 0; i < 4; i++) {
      cost->AddParameterBlock(6);
    }
    cost->AddParameterBlock(3);
    cost->AddParameterBlock(3);
    cost->AddParameterBlock(3);
    cost->SetNumResiduals(6);
    return cost;
  }

  PoseSpline spline;
  double time;
  Eigen::Vector3d gyro_measurement;
  Eigen::Vector3d accel_measurement;
  double gyro_sigma;
  double accel_sigma;
};

struct BiasPrior
{
  BiasPrior(const Eigen::Vector3d & prior, double sigma)
  : prior(prior), sigma(sigma) {}

  template<typename T>
  bool operator()(const T * value, T * residual) const
  {
    for (int i = 0; i < 3; i++) {
      residual[i] = (value[i] - T(prior[i])) / T(sigma);
    }
    return true;
  }

  static ceres::CostFunction * Create(const Eigen::Vector3d & prior, double sigma)
  {
    return new ceres::AutoDiffCostFunction<BiasPrior, 3, 3>(
      new BiasPrior(prior, sigma));
  }

  Eigen::Vector3d prior;
  double sigma;
};

struct TimeOffsetPrior
{
  TimeOffsetPrior(double prior, double sigma)
  : prior(prior), sigma(sigma) {}

  template<typename T>
  bool operator()(const T * value, T * residual) const
  {
    residual[0] = (value[0] - T(prior)) / T(sigma);
    return true;
  }

  static ceres::CostFunction * Create(double prior, double sigma)
  {
    return new ceres::AutoDiffCostFunction<TimeOffsetPrior, 1, 1>(
      new TimeOffsetPrior(prior, sigma));
  }

  double prior;
  double sigma;
};

struct GravityNormPrior
{
  GravityNormPrior(double magnitude, double sigma)
  : magnitude(magnitude), sigma(sigma) {}

  template<typename T>
  bool operator()(const T * gravity, T * residual) const
  {
    const T norm = sqrt(
      gravity[0] * gravity[0] + gravity[1] * gravity[1] + gravity[2] * gravity[2]);
    residual[0] = (norm - T(magnitude)) / T(sigma);
    return true;
  }

  static ceres::CostFunction * Create(double magnitude, double sigma)
  {
    return new ceres::AutoDiffCostFunction<GravityNormPrior, 1, 3>(
      new GravityNormPrior(magnitude, sigma));
  }

  double magnitude;
  double sigma;
};

}  // namespace ssct_cam_imu_calib
