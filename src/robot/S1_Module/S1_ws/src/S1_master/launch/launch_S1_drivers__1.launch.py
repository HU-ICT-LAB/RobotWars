"""This file launches the functionalities for the ROS2 package 'S1_Module'."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """This function contains the launch description."""
    return LaunchDescription([
        Node(
            package='S1_master',
            namespace='',
            executable='S1_module',
            output='screen'
        ),
    ])
