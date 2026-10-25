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

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <string>

#include "ssct_common/calib_param_manager.hpp"

namespace ssct_common
{
namespace
{

void write_file(const std::string & path, const std::string & content)
{
  std::ofstream ofs(path);
  ofs << content;
}

TEST(CalibParamManager, load_rotation_matrix)
{
  std::string path = "test_rotation_matrix.yaml";
  write_file(
    path,
    "transforms:\n"
    "    transform1:\n"
    "        frame_id: a\n"
    "        child_frame_id: b\n"
    "        translation: [1, 2, 3]\n"
    "        rotation: [0, -1, 0, 1, 0, 0, 0, 0, 1]\n");
  CalibParamManager manager;
  ASSERT_TRUE(manager.load(path));
  std::remove(path.c_str());
  ExtrinsicParam param;
  ASSERT_TRUE(manager.get_extrinsic_param("a", "b", param));
  EXPECT_NEAR(param.transform(0, 3), 1.0, 1e-9);
  EXPECT_NEAR(param.transform(1, 3), 2.0, 1e-9);
  EXPECT_NEAR(param.transform(2, 3), 3.0, 1e-9);
  EXPECT_NEAR(param.transform(0, 0), 0.0, 1e-9);
  EXPECT_NEAR(param.transform(0, 1), -1.0, 1e-9);
  EXPECT_NEAR(param.transform(1, 0), 1.0, 1e-9);
  EXPECT_NEAR(param.transform(1, 1), 0.0, 1e-9);
}

TEST(CalibParamManager, prefer_rotation_xyzw)
{
  std::string path = "test_rotation_xyzw.yaml";
  write_file(
    path,
    "transforms:\n"
    "    transform1:\n"
    "        frame_id: a\n"
    "        child_frame_id: b\n"
    "        translation: [0, 0, 0]\n"
    "        rotation: [1, 0, 0, 0, 1, 0, 0, 0, 1]\n"
    "        rotation_xyzw: [0, 0, 0.7071067811865476, 0.7071067811865476]\n");
  CalibParamManager manager;
  ASSERT_TRUE(manager.load(path));
  std::remove(path.c_str());
  ExtrinsicParam param;
  ASSERT_TRUE(manager.get_extrinsic_param("a", "b", param));
  // rotation_xyzw (90 deg about z) must win over the identity rotation matrix
  EXPECT_NEAR(param.transform(0, 0), 0.0, 1e-9);
  EXPECT_NEAR(param.transform(0, 1), -1.0, 1e-9);
  EXPECT_NEAR(param.transform(1, 0), 1.0, 1e-9);
}

TEST(CalibParamManager, save_rotation_xyzw_only)
{
  std::string path = "test_save_rotation.yaml";
  CalibParamManager manager;
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  manager.add_extrinsic_param("a", "b", transform);
  ASSERT_TRUE(manager.save(path));
  std::ifstream ifs(path);
  std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
  ifs.close();
  std::remove(path.c_str());
  EXPECT_NE(content.find("rotation_xyzw:"), std::string::npos);
  EXPECT_EQ(content.find("rotation:"), std::string::npos);
}

TEST(CalibParamManager, camera_model_aliases_and_coeff_counts)
{
  CalibParamManager manager;
  CameraIntrinsicParam camera;
  camera.frame_id = "camera";
  camera.width = 640;
  camera.height = 480;
  camera.type = "fisheye";
  camera.intrinsics = {500.0, 500.0, 320.0, 240.0};
  camera.distortion_coeffs = {0.1, 0.01, 0.001, 0.0001};
  ASSERT_TRUE(manager.add_camera_intrinsic_param(camera));
  CameraIntrinsicParam result;
  ASSERT_TRUE(manager.get_camera_intrinsic_param("camera", result));
  EXPECT_EQ(result.type, "ftheta_kb");

  camera.type = "omnidir";
  camera.intrinsics = {0.5, 500.0, 500.0, 320.0, 240.0};
  camera.distortion_coeffs = {0.1, 0.01, 0.001, 0.0001};
  ASSERT_TRUE(manager.add_camera_intrinsic_param(camera));
  ASSERT_TRUE(manager.get_camera_intrinsic_param("camera", result));
  EXPECT_EQ(result.type, "mei_radtan");

  camera.type = "pinhole_radtan";
  camera.intrinsics = {500.0, 500.0, 320.0, 240.0};
  camera.distortion_coeffs = {0.1, 0.01, 0.001, 0.0001};
  EXPECT_FALSE(manager.add_camera_intrinsic_param(camera));
}

}  // namespace
}  // namespace ssct_common
