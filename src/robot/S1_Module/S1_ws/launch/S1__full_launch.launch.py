"""All service calls."""
# """This file contains a function that describes how the nodes inside the teleop_joy and S1_master packages should be launched."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """All service calls."""
    S1_driver_1 = IncludeLaunchDescription(

        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('S1_master'), 'launch'),
            '/launch_S1_drivers__1.launch.py'])
    )

    teleop_joy_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('teleop_joy'), 'launch'),
            '/launch_teleop_joy_twist.launch.py'])
    )

    return LaunchDescription([
        S1_driver_1,
        teleop_joy_nodes
    ])
