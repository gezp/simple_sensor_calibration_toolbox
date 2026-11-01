# Copyright 2026 Gezp (https://github.com/gezp).
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
    data_dir = os.path.join(os.environ['HOME'], 'calibration_data')
    package_dir = get_package_share_directory('ssct_cam_imu_calib')
    calibration_config = os.path.join(package_dir, 'config', 'calibration.yaml')
    initial_calibration = os.path.join(package_dir, 'resource', 'initial_calibration.yaml')
    output_file = os.path.join(data_dir, 'cam_imu_result.yaml')
    calibration_node = Node(
        name='cam_imu_calibration_node',
        package='ssct_cam_imu_calib',
        executable='calibration_node',
        parameters=[
            {
                'camera_frame_id': 'camera',
                'imu_frame_id': 'imu',
                'calibration_config': calibration_config,
                'initial_calibration_file': initial_calibration,
                'output_file': output_file,
                'enable_compressed_image': True,
            }
        ],
        remappings=[
            ('image', '/camera/image_raw/compressed'),
            ('imu', '/imu/data'),
        ],
        output='screen',
    )
    return LaunchDescription([calibration_node])
