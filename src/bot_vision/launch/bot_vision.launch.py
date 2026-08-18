import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file_default = os.path.join(
        get_package_share_directory('bot_vision'),
        'config',
        'bot_vision_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        DeclareLaunchArgument(
            'params_file',
            default_value=params_file_default,
            description='Full path to the bot_vision parameters file'),

        # NOTE: The camera stream itself (topic /following_bot/camera/rgb)
        # is published by the following_bot package's Gazebo camera/bridge.
        # It is expected to already be running (e.g. via following_bot's
        # own launch file) before this launch file is started. This launch
        # file only starts the vision pipeline nodes that consume that
        # stream.
        Node(
            package='bot_vision',
            executable='yolo_detector',
            name='yolo_detector',
            parameters=[
                LaunchConfiguration('params_file'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            output='screen'),

        Node(
            package='bot_vision',
            executable='target_selector',
            name='target_selector',
            parameters=[
                LaunchConfiguration('params_file'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            output='screen'),

        Node(
            package='bot_vision',
            executable='tracker_node',
            name='tracker_node',
            parameters=[
                LaunchConfiguration('params_file'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            output='screen'),

        Node(
            package='bot_vision',
            executable='follow_controller',
            name='follow_controller',
            parameters=[
                LaunchConfiguration('params_file'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            output='screen'),
    ])
