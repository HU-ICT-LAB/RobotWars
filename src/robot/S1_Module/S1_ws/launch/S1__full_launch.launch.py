""""Launch file to start all nodes containes in packages 'teleop_joy' and 'S1_Master'"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    """The launch description function"""
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
