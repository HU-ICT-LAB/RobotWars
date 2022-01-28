"""This file contains the launch function for the package 'teleop_joy'."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """This function launches the nodes contained in the package 'teleop_joy'."""
    return LaunchDescription([
        Node(
            package="teleop_joy",
            executable="joy_pub",
            output="screen"
        ),
        Node(
            package="teleop_joy",
            executable="twist_pub",
            parameters=[
                {'linear_speed__x': 35.0},
                {'linear_speed__y': 35.0},
                {'angular_speed__pitch': 35.0},
                {'angular_speed__yaw': 35.0}
            ],
            output="screen"
        ),
    ])
