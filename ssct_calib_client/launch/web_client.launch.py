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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    host = LaunchConfiguration('host')
    port = LaunchConfiguration('port')
    mode = LaunchConfiguration('mode')
    return LaunchDescription(
        [
            DeclareLaunchArgument('host', default_value='0.0.0.0'),
            DeclareLaunchArgument('port', default_value='8080'),
            DeclareLaunchArgument(
                'mode',
                default_value='generic',
                description='generic, camera_intrinsic or lidar_cam_manual',
            ),
            Node(
                package='ssct_calib_client',
                executable='web_client.py',
                name='ssct_calib_client',
                parameters=[
                    {
                        'host': ParameterValue(host, value_type=str),
                        'port': ParameterValue(port, value_type=int),
                        'mode': ParameterValue(mode, value_type=str),
                    }
                ],
                output='screen',
            ),
        ]
    )
