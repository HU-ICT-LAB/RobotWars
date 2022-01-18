from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='S1_drivers',
            namespace='',
            executable='S1_module',
            output='screen'
        ),
    ])
