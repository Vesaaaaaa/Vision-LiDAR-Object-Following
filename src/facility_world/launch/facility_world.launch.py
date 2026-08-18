from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory("facility_world")
    
    return LaunchDescription([
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=os.path.join(pkg_share, "models")
        ),
        ExecuteProcess(
            cmd=[
                "gz",
                "sim",
                "-r",
                os.path.join(pkg_share, "worlds", "facility.world")
            ],
            additional_env={"GZ_SIM_SYSTEM_PLUGIN_PATH": "/opt/ros/jazzy/lib"},
            output="screen"
        )
    ])