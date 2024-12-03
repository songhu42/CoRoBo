from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="corobo_py", executable="crbs_mani"),
        Node(package="corobo_py", executable="crbs_server")
        ])
