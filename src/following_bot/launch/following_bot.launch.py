#!/usr/bin/env python3

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration 
from launch_ros.actions import Node


def generate_launch_description():
    following_bot_share = get_package_share_directory('following_bot')
    xacro_path = os.path.join(following_bot_share, 'urdf', 'following_bot.urdf.xacro')
    robot_description = xacro.process_file(xacro_path).toxml()
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }])

    spawn_following_bot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'following_bot',
            '-z', '0.1'
        ])

    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', 
            '/controller_manager'
        ])

    spawn_diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'diff_drive_controller',
            '--controller-manager', 
            '/controller_manager'
        ])

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='following_bot_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/following_bot/camera/rgb@sensor_msgs/msg/Image[gz.msgs.Image',
            '/following_bot/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ])
    
    delay_joint_state = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_following_bot,
            on_exit=[spawn_joint_state_broadcaster],
        )
    )
    
    delay_diff_drive = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_joint_state_broadcaster,
            on_exit=[spawn_diff_drive_controller],
        )
    )

    return LaunchDescription([
        declare_use_sim_time,
        robot_state_publisher,
        bridge,
        delay_joint_state,
        delay_diff_drive,
        spawn_following_bot
    ])
