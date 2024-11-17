#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
#
# Authors: Darby Lim

import os
import socket

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    tb3_name = LaunchConfiguration('tb3_name', default='robot1')

    usb_port = LaunchConfiguration('usb_port', default='/dev/ttyACM0')

    tb3_param_dir = LaunchConfiguration(
        'tb3_param_dir',
        default=os.path.join(
            get_package_share_directory('turtlebot3_bringup'),
            'param',
            'roslam_robots.yaml'))

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument('tb3_name', default_value=tb3_name),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'usb_port',
            default_value=usb_port,
            description='Connected USB port with OpenCR'),

        DeclareLaunchArgument(
            'tb3_param_dir',
            default_value=tb3_param_dir,
            description='Full path to turtlebot3 parameter file to load'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/utv_turtlebot3_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        Node(
            package='turtlebot3_node',
            executable='turtlebot3_ros',
            namespace=tb3_name,
            remappings=[
                ('/cmd_vel', ['/', tb3_name, '/cmd_vel']),
                ('/odom', ['/', tb3_name, '/odom']),
                ('/joint_states', ['/', tb3_name, '/joint_states']),
                ('/imu', ['/', tb3_name, '/imu']),
                ('/battery_state', ['/', tb3_name, '/battery_state']),
                ('/sensor_state', ['/', tb3_name, '/sensor_state']),
                ('/magnetic_field', ['/', tb3_name, '/magnetic_field']),
                ('/get_position', ['/', tb3_name, '/get_position']),
                ('/sound', ['/', tb3_name, '/sound']),
                ('/motor_power', ['/', tb3_name, '/motor_power']),
                ('/reset', ['/', tb3_name, '/reset'])
            ],
            parameters=[tb3_param_dir],
            arguments=['-i', usb_port],
            output='screen'),
    ])
