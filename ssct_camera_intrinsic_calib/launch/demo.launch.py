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
    camera_data_dir = os.path.join(data_dir, "camera_intrinsic")
    pkg_ssct_camera_intrinsic_calib = get_package_share_directory(
        "ssct_camera_intrinsic_calib"
    )
    calibrator_config = os.path.join(
        pkg_ssct_camera_intrinsic_calib, "config", "calibrator.yaml"
    )
    calibration_result = os.path.join(
        os.environ["HOME"], "calibration_data", "result.yaml"
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
        name="camera_intrisic_calibration_node",
        package="ssct_camera_intrinsic_calib",
        executable="calibration_node",
        parameters=[
            {
                "frame_id": "center_camera",
                "calibrator_config": calibrator_config,
                "output_file": calibration_result,
            }
        ],
        remappings=[("image", "/sensor/center_camera/image")],
        output="screen",
    )
    calibration_client = Node(
        name="calibration_client",
        package="ssct_camera_intrinsic_calib",
        executable="calibration_client.py",
        output="screen",
    )
    ld = LaunchDescription()
    ld.add_action(dummy_camera_node)
    ld.add_action(calibration_node)
    ld.add_action(calibration_client)
    return ld
