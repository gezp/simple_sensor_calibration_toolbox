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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TARGET_FRAME_ID = 'top_center_lidar'


def launch_setup(context, *args, **kwargs):
    source = LaunchConfiguration('source').perform(context)
    source_frame_id = f'{source}_lidar'
    data_dir = os.path.join(
        os.environ['HOME'], 'calibration_data', 'SensorsCalibration', 'lidar2lidar'
    )
    pkg_share = get_package_share_directory('ssct_lidar_lidar_calib')
    calibration_config = os.path.join(pkg_share, 'config', 'manual_calibration.yaml')
    initial_calibration = os.path.join(
        pkg_share, 'config', 'initial_calibration.yaml'
    )
    output_calibration = os.path.join(
        os.environ['HOME'], 'calibration_data', 'result.yaml'
    )
    target_lidar_node = Node(
        name='dummy_top_center_lidar_node',
        package='ssct_common',
        executable='dummy_lidar_node',
        parameters=[
            {
                'data_dir': os.path.join(data_dir, 'top'),
                'frame_id': TARGET_FRAME_ID,
                'rate': 1.0,
            }
        ],
        output='screen',
    )
    source_lidar_node = Node(
        name=f'dummy_{source}_lidar_node',
        package='ssct_common',
        executable='dummy_lidar_node',
        parameters=[
            {
                'data_dir': os.path.join(data_dir, source),
                'frame_id': source_frame_id,
                'rate': 1.0,
            }
        ],
        output='screen',
    )
    manual_calibration_node = Node(
        name='lidar_lidar_manual_calibration_node',
        package='ssct_lidar_lidar_calib',
        executable='manual_calibration_node',
        parameters=[
            {
                'target_frame_id': TARGET_FRAME_ID,
                'source_frame_id': source_frame_id,
                'calibration_config': calibration_config,
                'initial_calibration_file': initial_calibration,
                'output_calibration_file': output_calibration,
            }
        ],
        remappings=[
            ('target_pointcloud', f'/sensor/{TARGET_FRAME_ID}/pointcloud'),
            ('source_pointcloud', f'/sensor/{source_frame_id}/pointcloud'),
        ],
        output='screen',
    )
    return [target_lidar_node, source_lidar_node, manual_calibration_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'source',
                default_value='left',
                choices=['left', 'right'],
                description='source lidar to calibrate against top_center_lidar',
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
