import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config = LaunchConfiguration('rviz_config')
    use_rviz = LaunchConfiguration('use_rviz')

    facility_world_share = get_package_share_directory('facility_world')
    following_bot_share = get_package_share_directory('following_bot')
    bot_vision_share = get_package_share_directory('bot_vision')
    obstacle_avoidance_share = get_package_share_directory('obstacle_avoidance')
    bringup_share = get_package_share_directory('bringup')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock if true')

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Whether to start RViz2')

    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(bringup_share, 'rviz', 'bringup.rviz'),
        description='Path to the RViz config file')

    facility_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(facility_world_share, 'launch', 'facility_world.launch.py')))

    following_bot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(following_bot_share, 'launch', 'following_bot.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items())

    bot_vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bot_vision_share, 'launch', 'bot_vision.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items())

    obstacle_avoidance_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(obstacle_avoidance_share, 'launch', 'obstacle_avoidance.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items())

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz))

    return LaunchDescription([
        declare_use_sim_time,
        declare_use_rviz,
        declare_rviz_config,
        facility_world_launch,
        following_bot_launch,
        bot_vision_launch,
        obstacle_avoidance_launch,
        rviz_node,
    ])
