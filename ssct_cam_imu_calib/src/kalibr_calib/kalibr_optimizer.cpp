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

#include "ssct_cam_imu_calib/kalibr_calib/kalibr_optimizer.hpp"

#include <ceres/ceres.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <opencv2/imgproc.hpp>
#include <sophus/so3.hpp>

#include "ssct_cam_imu_calib/kalibr_calib/kalibr_factor.hpp"
#include "ssct_cam_imu_calib/kalibr_calib/kalibr_initialization.hpp"
#include "ssct_cam_imu_calib/kalibr_calib/pose_spline.hpp"

namespace ssct_cam_imu_calib
{
namespace
{

// Minimum corners needed for a board pose.
constexpr int kMinBoardCorners = 4;

// Robust loss for the camera and IMU residual blocks.
constexpr double kHuberThreshold = 1.0;

// Time-offset grid search and the sigma floor used by the noise model.
constexpr double kTimeOffsetSearchStep = 0.002;
constexpr double kMinReprojectionSigma = 1e-6;
constexpr double kMinImuSigma = 1e-9;

// IMU sample interval clamp, used to convert noise density to discrete sigma.
constexpr double kMinSampleDt = 1e-4;
constexpr double kMaxSampleDt = 0.1;
constexpr double kDefaultSampleDt = 0.005;

// A frame is reported as bad above this RMS.
constexpr double kBadFrameRmsPx = 5.0;

constexpr double kFunctionTolerance = 1e-8;
constexpr double kGradientTolerance = 1e-10;
constexpr double kParameterTolerance = 1e-8;

std::string to_string(const KalibrEstimatedState & state)
{
  std::ostringstream stream;
  stream.setf(std::ios::fixed, std::ios::floatfield);
  stream.precision(6);
  const Eigen::Quaterniond rotation(state.T_camera_imu.block<3, 3>(0, 0));
  stream << "time offset " << state.time_offset << " s"
         << ", translation [" << state.T_camera_imu(0, 3) << ", "
         << state.T_camera_imu(1, 3) << ", " << state.T_camera_imu(2, 3) << "]"
         << ", rotation_xyzw [" << rotation.x() << ", " << rotation.y() << ", "
         << rotation.z() << ", " << rotation.w() << "]"
         << ", gyro bias [" << state.gyro_bias.x() << ", " << state.gyro_bias.y()
         << ", " << state.gyro_bias.z() << "]"
         << ", accel bias [" << state.accel_bias.x() << ", " << state.accel_bias.y()
         << ", " << state.accel_bias.z() << "]"
         << ", gravity [" << state.gravity.x() << ", " << state.gravity.y()
         << ", " << state.gravity.z() << "]";
  return stream.str();
}

std::string to_string(const Evaluation & evaluation)
{
  std::ostringstream stream;
  stream.setf(std::ios::fixed, std::ios::floatfield);
  stream.precision(6);
  stream << "reprojection rms " << evaluation.reprojection_rms << " px"
         << ", reprojection median " << evaluation.reprojection_median << " px"
         << ", p95 " << evaluation.reprojection_p95 << " px"
         << ", bad frames " << evaluation.bad_frames
         << ", gyro rms " << evaluation.gyro_rms << " rad/s"
         << ", accel rms " << evaluation.accel_rms << " m/s^2";
  return stream.str();
}

}  // namespace

KalibrOptimizer::KalibrOptimizer(const YAML::Node & config)
{
  max_iterations_ = config["max_iterations"].as<int>();
  spline_knot_spacing_ = config["spline_knot_spacing"].as<double>();
  reprojection_sigma_px_ = config["reprojection_sigma_px"].as<double>();
  max_reprojection_rms_ = config["max_reprojection_rms"].as<double>();

  gyro_bias_sigma_ = config["gyro_bias_sigma"].as<double>();
  accel_bias_sigma_ = config["accel_bias_sigma"].as<double>();
  time_offset_sigma_ = config["time_offset_sigma"].as<double>();
  max_time_offset_ =
    std::max(0.0, config["max_time_offset"].as<double>());
  gravity_sigma_ = config["gravity_sigma"].as<double>();
  estimate_time_offset_ = config["estimate_time_offset"].as<bool>();
  estimate_gravity_ = config["estimate_gravity"].as<bool>();
}

void KalibrOptimizer::set_camera_model(const CameraModel & camera_model)
{
  camera_model_ = &camera_model;
}

void KalibrOptimizer::set_imu_intrinsic_param(
  const ssct_common::ImuIntrinsicParam & imu)
{
  gyro_noise_density_ = imu.gyro_noise_density.mean();
  accel_noise_density_ = imu.accel_noise_density.mean();
  if (gyro_noise_density_ <= 0.0 || accel_noise_density_ <= 0.0) {
    throw std::runtime_error("IMU noise density is required");
  }
}

void KalibrOptimizer::add_data(
  const std::vector<BoardObservation> & observations,
  const std::vector<ssct_common::ImuData> & imu_samples)
{
  // The optimizer only consumes the dynamic segment covered by camera frames;
  // samples outside the observed span constrain nothing. No still start is
  // required.
  double observation_start = std::numeric_limits<double>::infinity();
  double observation_end = -std::numeric_limits<double>::infinity();
  for (const auto & observation : observations) {
    observation_start = std::min(observation_start, observation.time);
    observation_end = std::max(observation_end, observation.time);
  }
  if (observations.empty()) {
    observations_.clear();
    imu_samples_.clear();
    return;
  }

  // Rebase timestamps to the first observation. Absolute ROS time is ~1.8e9 s,
  // whose double resolution is ~0.2 us: numeric differentiation of the time
  // offset then sees no change at all and the offset is never optimized.
  const double time_origin = observation_start;
  observations_ = observations;
  for (auto & observation : observations_) {
    observation.time -= time_origin;
  }
  imu_samples_.clear();
  imu_samples_.reserve(imu_samples.size());
  for (const auto & sample : imu_samples) {
    if (sample.time < observation_start || sample.time > observation_end) {
      continue;
    }
    imu_samples_.push_back(sample);
    imu_samples_.back().time -= time_origin;
  }
}

bool KalibrOptimizer::optimize()
{
  KalibrEstimatedState initial_state;
  if (!build_initial_guess(initial_state)) {
    return false;
  }

  const auto initial_evaluation = evaluate(initial_state);

  KalibrEstimatedState final_state;
  const bool solved = solve_problem(initial_state, final_state);

  result_.transform = final_state.T_camera_imu;
  result_.time_offset = final_state.time_offset;
  if (!result_.transform.allFinite() || !std::isfinite(result_.time_offset)) {
    message_ = "non-finite optimization result";
    return false;
  }

  const auto final_evaluation = evaluate(final_state);

  const bool success = solved &&
    std::isfinite(final_evaluation.reprojection_rms) &&
    final_evaluation.reprojection_rms <= max_reprojection_rms_;
  std::ostringstream stream;
  stream.setf(std::ios::fixed, std::ios::floatfield);
  stream.precision(6);
  stream << "kalibr: frames " << observations_.size()
         << ", controls " << final_state.controls.size()
         << ", " << solver_message_
         << "\ninitial evaluation: " << to_string(initial_evaluation)
         << "\nfinal evaluation: " << to_string(final_evaluation)
         << "\ninitial state: " << to_string(initial_state)
         << "\nfinal state: " << to_string(final_state);
  message_ = stream.str();
  return success;
}

bool KalibrOptimizer::build_initial_guess(KalibrEstimatedState & guess)
{
  // Kalibr-style initialization order:
  // 1. time offset from the angular speed magnitude (rotation invariant),
  // 2. camera-IMU rotation and gyro bias from the gyro/vision orientation fit,
  // 3. pose spline and gravity direction from the board observations.
  double time_offset = 0.0;
  if (estimate_time_offset_) {
    time_offset = estimate_time_offset(
      observations_, imu_samples_, max_time_offset_, kTimeOffsetSearchStep);
  }

  Eigen::Matrix3d R_body_camera;
  Eigen::Vector3d init_gyro_bias;
  if (!estimate_orientation_prior(
      observations_, imu_samples_, time_offset, Eigen::Matrix3d::Identity(),
      R_body_camera, init_gyro_bias))
  {
    message_ = "failed to initialize camera-IMU rotation prior";
    return false;
  }

  // The translation is not observable from gyro/vision; it starts at zero.
  guess.T_camera_imu.setIdentity();
  guess.T_camera_imu.block<3, 3>(0, 0) = R_body_camera.transpose();
  if (!guess.T_camera_imu.allFinite()) {
    message_ = "invalid initial camera-IMU extrinsic";
    return false;
  }
  guess.time_offset = time_offset;
  guess.gyro_bias = init_gyro_bias;

  if (!build_pose_spline(
      observations_, imu_samples_, guess.T_camera_imu, time_offset,
      max_time_offset_, spline_knot_spacing_, kStandardGravity,
      guess.spline, guess.controls, guess.gravity))
  {
    message_ = "failed to initialize pose spline";
    return false;
  }
  return true;
}

bool KalibrOptimizer::solve_problem(
  const KalibrEstimatedState & initial_state,
  KalibrEstimatedState & final_state)
{
  // Evaluation window for the segment lookup; must match the time-offset bounds
  // below.
  double offset_min = initial_state.time_offset;
  double offset_max = initial_state.time_offset;
  if (estimate_time_offset_) {
    offset_min = -max_time_offset_;
    offset_max = max_time_offset_;
  }

  final_state = initial_state;
  Eigen::Matrix<double, 6, 1> camera_imu;
  camera_imu.head<3>() = Sophus::SO3d(initial_state.T_camera_imu.block<3, 3>(0, 0)).log();
  camera_imu.tail<3>() = initial_state.T_camera_imu.block<3, 1>(0, 3);
  double time_offset = initial_state.time_offset;
  Eigen::Vector3d gyro_bias = initial_state.gyro_bias;
  Eigen::Vector3d accel_bias = initial_state.accel_bias;
  Eigen::Vector3d gravity = initial_state.gravity;

  ceres::Problem problem;
  for (auto & control : final_state.controls) {
    problem.AddParameterBlock(control.data(), 6);
  }
  problem.AddParameterBlock(camera_imu.data(), 6);
  problem.AddParameterBlock(&time_offset, 1);
  problem.AddParameterBlock(gyro_bias.data(), 3);
  problem.AddParameterBlock(accel_bias.data(), 3);
  problem.AddParameterBlock(gravity.data(), 3);
  if (estimate_time_offset_) {
    problem.SetParameterLowerBound(&time_offset, 0, -max_time_offset_);
    problem.SetParameterUpperBound(&time_offset, 0, max_time_offset_);
  } else {
    problem.SetParameterBlockConstant(&time_offset);
  }
  if (!estimate_gravity_) {
    problem.SetParameterBlockConstant(gravity.data());
  }
  // The outer control points are outside the measurement span and only provide
  // boundary support.
  problem.SetParameterBlockConstant(final_state.controls.front().data());
  problem.SetParameterBlockConstant(final_state.controls.back().data());

  const double focal_x = camera_model_->focal_x();
  const double focal_y = camera_model_->focal_y();
  const double reprojection_sigma = std::max(kMinReprojectionSigma, reprojection_sigma_px_);
  const int num_controls = static_cast<int>(final_state.controls.size());
  for (const auto & observation : observations_) {
    // A cubic segment s needs controls [s - 1, s + 4]; the window covers every
    // segment the unknown time offset can select.
    const int first_segment = final_state.spline.segment_index(observation.time + offset_min);
    const int last_segment = final_state.spline.segment_index(observation.time + offset_max);
    const int first_control = std::max(0, first_segment - (PoseSpline::kOrder - 3));
    const int last_control = std::min(num_controls - 1, last_segment + PoseSpline::kOrder);
    const int control_count = last_control - first_control + 1;
    if (control_count < kMinBoardCorners) {
      continue;
    }
    std::vector<double *> parameter_blocks;
    parameter_blocks.reserve(static_cast<size_t>(control_count) + 2);
    for (int i = first_control; i <= last_control; i++) {
      parameter_blocks.push_back(final_state.controls[i].data());
    }
    parameter_blocks.push_back(camera_imu.data());
    parameter_blocks.push_back(&time_offset);
    for (size_t j = 0; j < observation.object_points.size(); j++) {
      auto * cost = ReprojectionFactor::Create(
        final_state.spline, first_control, control_count, observation.time,
        Eigen::Vector3d(
          observation.object_points[j].x, observation.object_points[j].y,
          observation.object_points[j].z),
        observation.normalized_points[j], focal_x, focal_y, reprojection_sigma);
      problem.AddResidualBlock(
        cost, new ceres::HuberLoss(kHuberThreshold), parameter_blocks);
    }
  }

  for (size_t i = 0; i < imu_samples_.size(); i++) {
    const double dt = i + 1 < imu_samples_.size() ?
      imu_samples_[i + 1].time - imu_samples_[i].time :
      (i > 0 ? imu_samples_[i].time - imu_samples_[i - 1].time : kDefaultSampleDt);
    const double sample_dt = std::max(kMinSampleDt, std::min(kMaxSampleDt, dt));
    const double gyro_sigma = std::max(
      kMinImuSigma, gyro_noise_density_ / std::sqrt(sample_dt));
    const double accel_sigma = std::max(
      kMinImuSigma, accel_noise_density_ / std::sqrt(sample_dt));
    const int segment = final_state.spline.segment_index(imu_samples_[i].time);
    std::vector<double *> parameter_blocks = {
      final_state.controls[segment].data(), final_state.controls[segment + 1].data(),
      final_state.controls[segment + 2].data(), final_state.controls[segment + 3].data(),
      gyro_bias.data(), accel_bias.data(), gravity.data()};
    auto * cost = ImuFactor::Create(
      final_state.spline, imu_samples_[i].time, imu_samples_[i].angular_velocity,
      imu_samples_[i].linear_acceleration, gyro_sigma, accel_sigma);
    problem.AddResidualBlock(
      cost, new ceres::HuberLoss(kHuberThreshold), parameter_blocks);
  }

  if (gyro_bias_sigma_ > 0.0) {
    auto * cost = BiasPrior::Create(initial_state.gyro_bias, gyro_bias_sigma_);
    problem.AddResidualBlock(cost, nullptr, gyro_bias.data());
  }
  if (accel_bias_sigma_ > 0.0) {
    auto * cost = BiasPrior::Create(initial_state.accel_bias, accel_bias_sigma_);
    problem.AddResidualBlock(cost, nullptr, accel_bias.data());
  }
  if (estimate_time_offset_ && time_offset_sigma_ > 0.0) {
    auto * cost = TimeOffsetPrior::Create(time_offset, time_offset_sigma_);
    problem.AddResidualBlock(cost, nullptr, &time_offset);
  }
  if (estimate_gravity_ && gravity_sigma_ > 0.0) {
    auto * cost = GravityNormPrior::Create(kStandardGravity, gravity_sigma_);
    problem.AddResidualBlock(cost, nullptr, gravity.data());
  }
  ceres::Solver::Options options;
  options.max_num_iterations = max_iterations_;
  options.function_tolerance = kFunctionTolerance;
  options.gradient_tolerance = kGradientTolerance;
  options.parameter_tolerance = kParameterTolerance;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.minimizer_progress_to_stdout = false;
  options.logging_type = ceres::SILENT;
  options.num_threads = 1;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  final_state.T_camera_imu.setIdentity();
  final_state.T_camera_imu.block<3, 3>(0, 0) =
    Sophus::SO3d::exp(camera_imu.head<3>()).matrix();
  final_state.T_camera_imu.block<3, 1>(0, 3) = camera_imu.tail<3>();
  final_state.time_offset = time_offset;
  final_state.gyro_bias = gyro_bias;
  final_state.accel_bias = accel_bias;
  final_state.gravity = gravity;

  std::ostringstream stream;
  stream << "termination " << summary.termination_type
         << ", iterations " << summary.iterations.size();
  solver_message_ = stream.str();
  return summary.termination_type != ceres::FAILURE;
}

Evaluation KalibrOptimizer::evaluate(const KalibrEstimatedState & state) const
{
  Evaluation evaluation;
  std::vector<double> errors;
  for (const auto & observation : observations_) {
    PoseSample sample;
    if (!state.spline.evaluate(
        state.controls, observation.time + state.time_offset, sample))
    {
      continue;
    }
    const Eigen::Matrix4d T_camera_board =
      state.T_camera_imu * sample.T_world_body.inverse();
    std::vector<cv::Point2f> projected;
    if (!camera_model_->project_points(observation.object_points, T_camera_board, projected)) {
      continue;
    }
    double frame_squared_error = 0.0;
    for (size_t j = 0; j < projected.size(); j++) {
      const cv::Point2f diff = projected[j] - observation.image_points[j];
      const double error = std::hypot(diff.x, diff.y);
      errors.push_back(error);
      frame_squared_error += error * error;
    }
    if (!projected.empty()) {
      const double frame_rms =
        std::sqrt(frame_squared_error / static_cast<double>(projected.size()));
      if (frame_rms > kBadFrameRmsPx) {
        evaluation.bad_frames++;
      }
    }
  }
  if (!errors.empty()) {
    double squared_error = 0.0;
    for (const double error : errors) {
      squared_error += error * error;
    }
    evaluation.reprojection_rms = std::sqrt(squared_error / static_cast<double>(errors.size()));
    std::sort(errors.begin(), errors.end());
    evaluation.reprojection_median = errors[errors.size() / 2];
    evaluation.reprojection_p95 = errors[errors.size() * 95 / 100];
  }

  double gyro_squared_error = 0.0;
  double accel_squared_error = 0.0;
  size_t imu_error_count = 0;
  for (const auto & sample : imu_samples_) {
    const int segment = state.spline.segment_index(sample.time);
    const double * segment_controls[4] = {
      state.controls[segment].data(), state.controls[segment + 1].data(),
      state.controls[segment + 2].data(), state.controls[segment + 3].data()};
    PoseSample pose;
    if (!state.spline.evaluate(segment_controls, sample.time, pose)) {
      continue;
    }
    const Eigen::Vector3d gyro_error =
      pose.angular_velocity - (sample.angular_velocity - state.gyro_bias);
    const Eigen::Vector3d specific_force =
      pose.T_world_body.block<3, 3>(0, 0).transpose() *
      (pose.acceleration - state.gravity);
    const Eigen::Vector3d accel_error =
      specific_force - (sample.linear_acceleration - state.accel_bias);
    gyro_squared_error += gyro_error.squaredNorm();
    accel_squared_error += accel_error.squaredNorm();
    imu_error_count++;
  }
  evaluation.gyro_rms = imu_error_count > 0 ?
    std::sqrt(gyro_squared_error / static_cast<double>(imu_error_count)) : 0.0;
  evaluation.accel_rms = imu_error_count > 0 ?
    std::sqrt(accel_squared_error / static_cast<double>(imu_error_count)) : 0.0;
  return evaluation;
}

}  // namespace ssct_cam_imu_calib
