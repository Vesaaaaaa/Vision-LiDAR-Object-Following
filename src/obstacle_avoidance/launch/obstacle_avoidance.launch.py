#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('obstacle_avoidance'),
        'config',
        'obstacle_avoidance_params.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        Node(
            package='obstacle_avoidance',
            executable='obstacle_avoidance_node',
            name='obstacle_avoidance_node',
            parameters=[params_file, {
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
            output='screen'),
    ])
