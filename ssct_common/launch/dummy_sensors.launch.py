# Copyright 2024 Gezp (https://github.com/gezp).
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    data_dir = os.path.join(os.environ["HOME"], "calibration_data")
    lidar_data_dir = os.path.join(data_dir, "SensorsCalibration", "lidar2camera", "lidar")
    camera_data_dir = os.path.join(data_dir, "SensorsCalibration", "camera_intrinsic")
    imu_data_file = os.path.join(data_dir, "imu_data", "uncalibrated_imu_data.csv")
    dummy_lidar_node = Node(
        name="dummy_lidar_node",
        package="ssct_common",
        executable="dummy_lidar_node",
        parameters=[{"data_dir": lidar_data_dir, "frame_id": "top_center_lidar"}],
        output="screen",
    )
    dummy_camera_node = Node(
        name="dummy_camera_node",
        package="ssct_common",
        executable="dummy_camera_node",
        parameters=[
            {
                "data_dir": camera_data_dir,
                "frame_id": "center_camera",
                "rate": 1.0,
            }
        ],
        output="screen",
    )
    dummy_imu_node = Node(
        name="dummy_imu_node",
        package="ssct_common",
        executable="dummy_imu_node",
        parameters=[
            {
                "data_file": imu_data_file,
                "frame_id": "center_imu",
                "rate": 100.0,
            }
        ],
        output="screen",
    )
    ld = LaunchDescription()
    ld.add_action(dummy_lidar_node)
    ld.add_action(dummy_camera_node)
    ld.add_action(dummy_imu_node)
    return ld
