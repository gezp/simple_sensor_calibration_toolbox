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
    package_dir = get_package_share_directory('ssct_imu_intrinsic_calib')
    calibration_config = os.path.join(package_dir, 'config', 'imu_tk_calibration.yaml')
    output_file = os.path.join(
        os.environ['HOME'], 'calibration_data', 'result.yaml'
    )
    calibration_node = Node(
        name='imu_intrinsic_calibration_node',
        package='ssct_imu_intrinsic_calib',
        executable='calibration_node',
        parameters=[
            {
                'imu_frame_id': 'imu',
                'calibration_config': calibration_config,
                'output_file': output_file,
            }
        ],
        remappings=[('imu', '/imu/data')],
        output='screen',
    )
    return LaunchDescription([
        calibration_node,
    ])
