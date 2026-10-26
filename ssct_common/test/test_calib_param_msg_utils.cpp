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

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include "rclcpp/time.hpp"
#include "ssct_common/calib_param_msg_utils.hpp"

namespace ssct_common
{
namespace
{

TEST(CalibParamMsgUtils, extrinsic_round_trip)
{
  ExtrinsicParam param;
  param.frame_id = "target";
  param.child_frame_id = "source";
  param.transform = Eigen::Matrix4d::Identity();
  param.transform.block<3, 3>(0, 0) =
    Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  param.transform.block<3, 1>(0, 3) << 1.0, 2.0, 3.0;

  ExtrinsicParam result;
  ASSERT_TRUE(from_ros_msg(to_ros_msg(param), result));
  EXPECT_EQ(result.frame_id, param.frame_id);
  EXPECT_EQ(result.child_frame_id, param.child_frame_id);
  EXPECT_LT((result.transform - param.transform).norm(), 1e-9);
}

TEST(CalibParamMsgUtils, reject_invalid_extrinsic)
{
  ssct_interfaces::msg::ExtrinsicParam msg;
  msg.transform.rotation.w = 0.0;
  ExtrinsicParam param;
  EXPECT_FALSE(from_ros_msg(msg, param));
}

TEST(CalibParamMsgUtils, camera_round_trip)
{
  CameraIntrinsicParam param;
  param.frame_id = "camera";
  param.height = 720;
  param.width = 1280;
  param.type = "pinhole_radtan";
  param.intrinsics = {1000.0, 1001.0, 640.0, 360.0};
  param.distortion_coeffs = {-0.1, 0.01, 0.0, 0.0, 0.0};

  CameraIntrinsicParam result;
  ASSERT_TRUE(from_ros_msg(to_ros_msg(param), result));
  EXPECT_EQ(result.frame_id, param.frame_id);
  EXPECT_EQ(result.height, param.height);
  EXPECT_EQ(result.width, param.width);
  EXPECT_EQ(result.type, param.type);
  ASSERT_EQ(result.intrinsics.size(), param.intrinsics.size());
  for (size_t i = 0; i < param.intrinsics.size(); ++i) {
    EXPECT_NEAR(result.intrinsics[i], param.intrinsics[i], 1e-6);
  }
}

TEST(CalibParamMsgUtils, imu_round_trip)
{
  ImuIntrinsicParam param;
  param.frame_id = "imu";
  param.accel_matrix = Eigen::Matrix3d::Identity();
  param.accel_matrix(0, 1) = 0.01;
  param.accel_bias << 0.1, 0.2, 0.3;
  param.accel_noise_density << 0.001, 0.002, 0.003;
  param.accel_random_walk << 0.0001, 0.0002, 0.0003;
  param.gyro_matrix = Eigen::Matrix3d::Identity();
  param.gyro_matrix(1, 2) = -0.02;
  param.gyro_bias << -0.1, -0.2, -0.3;
  param.gyro_noise_density << 0.004, 0.005, 0.006;
  param.gyro_random_walk << 0.0004, 0.0005, 0.0006;

  ImuIntrinsicParam result;
  ASSERT_TRUE(from_ros_msg(to_ros_msg(param), result));
  EXPECT_EQ(result.frame_id, param.frame_id);
  EXPECT_LT((result.accel_matrix - param.accel_matrix).norm(), 1e-12);
  EXPECT_LT((result.accel_bias - param.accel_bias).norm(), 1e-12);
  EXPECT_LT((result.accel_noise_density - param.accel_noise_density).norm(), 1e-12);
  EXPECT_LT((result.accel_random_walk - param.accel_random_walk).norm(), 1e-12);
  EXPECT_LT((result.gyro_matrix - param.gyro_matrix).norm(), 1e-12);
  EXPECT_LT((result.gyro_bias - param.gyro_bias).norm(), 1e-12);
  EXPECT_LT((result.gyro_noise_density - param.gyro_noise_density).norm(), 1e-12);
  EXPECT_LT((result.gyro_random_walk - param.gyro_random_walk).norm(), 1e-12);
}

TEST(CalibParamMsgUtils, reject_invalid_imu)
{
  ssct_interfaces::msg::ImuIntrinsicParam msg;
  ImuIntrinsicParam param;
  EXPECT_FALSE(from_ros_msg(msg, param));
}

TEST(CalibParamMsgUtils, params_round_trip)
{
  CalibParamManager params;
  CameraIntrinsicParam camera;
  camera.frame_id = "camera";
  camera.height = 480;
  camera.width = 640;
  camera.type = "pinhole";
  camera.intrinsics = {500.0, 500.0, 320.0, 240.0};
  ASSERT_TRUE(params.add_camera_intrinsic_param(camera));
  ImuIntrinsicParam imu;
  imu.frame_id = "imu";
  imu.accel_matrix = Eigen::Matrix3d::Identity();
  imu.accel_bias.setZero();
  imu.accel_noise_density.setZero();
  imu.accel_random_walk.setZero();
  imu.gyro_matrix = Eigen::Matrix3d::Identity();
  imu.gyro_bias << 0.1, 0.2, 0.3;
  imu.gyro_noise_density.setZero();
  imu.gyro_random_walk.setZero();
  ASSERT_TRUE(params.add_imu_intrinsic_param(imu));
  ExtrinsicParam extrinsic;
  extrinsic.frame_id = "camera";
  extrinsic.child_frame_id = "lidar";
  extrinsic.transform = Eigen::Matrix4d::Identity();
  extrinsic.transform(0, 3) = 0.5;
  ASSERT_TRUE(params.add_extrinsic_param(extrinsic));

  auto msg = to_ros_msg(params, rclcpp::Time(12, 34));
  EXPECT_EQ(msg.timestamp.sec, 12);
  EXPECT_EQ(msg.timestamp.nanosec, 34u);

  CalibParamManager result;
  ASSERT_TRUE(from_ros_msg(msg, result));
  CameraIntrinsicParam result_camera;
  ASSERT_TRUE(result.get_camera_intrinsic_param("camera", result_camera));
  EXPECT_EQ(result_camera.width, camera.width);
  ImuIntrinsicParam result_imu;
  ASSERT_TRUE(result.get_imu_intrinsic_param("imu", result_imu));
  EXPECT_LT((result_imu.gyro_bias - imu.gyro_bias).norm(), 1e-12);
  ExtrinsicParam result_extrinsic;
  ASSERT_TRUE(result.get_extrinsic_param("camera", "lidar", result_extrinsic));
  EXPECT_NEAR(result_extrinsic.transform(0, 3), 0.5, 1e-9);
}

}  // namespace
}  // namespace ssct_common
