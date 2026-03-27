# Copyright 2026 chenzhaoqi
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

"""Launch file for voice_cmd_node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('linear_speed', default_value='0.2',
                              description='Linear speed (m/s)'),
        DeclareLaunchArgument('angular_speed', default_value='0.5',
                              description='Angular speed (rad/s)'),
        DeclareLaunchArgument('linear_distance', default_value='0.4',
                              description='Distance per forward/backward (m)'),
        DeclareLaunchArgument('angular_angle', default_value='90.0',
                              description='Angle per turn (degrees)'),
        DeclareLaunchArgument('odom_frame', default_value='odom',
                              description='Odometry frame'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint',
                              description='Base frame'),
        DeclareLaunchArgument('cmd_topic', default_value='voice_cmd',
                              description='Voice command topic (std_msgs/String)'),

        Node(
            package='jdbot_voice_control',
            executable='voice_cmd_node',
            name='voice_cmd_node',
            output='screen',
            parameters=[{
                'linear_speed':    LaunchConfiguration('linear_speed'),
                'angular_speed':   LaunchConfiguration('angular_speed'),
                'linear_distance': LaunchConfiguration('linear_distance'),
                'angular_angle':   LaunchConfiguration('angular_angle'),
                'odom_frame':      LaunchConfiguration('odom_frame'),
                'base_frame':      LaunchConfiguration('base_frame'),
                'cmd_topic':       LaunchConfiguration('cmd_topic'),
            }],
        ),
    ])
