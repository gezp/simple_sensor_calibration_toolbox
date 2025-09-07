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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    data_dir = os.path.join(
        os.environ["HOME"], "calibration_data", "SensorsCalibration"
    )
    lidar_data_dir = os.path.join(data_dir, "lidar2camera", "lidar")
    camera_data_dir = os.path.join(data_dir, "lidar2camera", "camera")
    pkg_ssct_lidar_cam_manual_calib = get_package_share_directory(
        "ssct_lidar_cam_manual_calib"
    )
    calibrator_config = os.path.join(
        pkg_ssct_lidar_cam_manual_calib, "config", "calibrator.yaml"
    )
    initial_calibration = os.path.join(
        pkg_ssct_lidar_cam_manual_calib, "config", "initial_calibration.yaml"
    )
    output_calibration = os.path.join(
        os.environ["HOME"], "calibration_data", "result.yaml"
    )
    dummy_lidar_node = Node(
        name="dummy_lidar_node",
        package="ssct_common",
        executable="dummy_lidar_node",
        parameters=[
            {
                "data_dir": lidar_data_dir,
                "frame_id": "top_center_lidar",
                "rate": 1.0,
            }
        ],
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
    calibration_node = Node(
        name="lidar_camera_calibration_node",
        package="ssct_lidar_cam_manual_calib",
        executable="calibration_node",
        parameters=[
            {
                "camera_frame_id": "center_camera",
                "lidar_frame_id": "top_center_lidar",
                "calibrator_config": calibrator_config,
                "initial_calibration_file": initial_calibration,
                "output_calibration_file": output_calibration,
            }
        ],
        remappings=[
            ("image", "/sensor/center_camera/image"),
            ("pointcloud", "/sensor/top_center_lidar/pointcloud"),
        ],
        output="screen",
    )
    calibration_client = Node(
        name="calibration_client",
        package="ssct_lidar_cam_manual_calib",
        executable="calibration_client.py",
        output="screen",
    )
    ld = LaunchDescription()
    ld.add_action(dummy_lidar_node)
    ld.add_action(dummy_camera_node)
    ld.add_action(calibration_node)
    ld.add_action(calibration_client)
    return ld
