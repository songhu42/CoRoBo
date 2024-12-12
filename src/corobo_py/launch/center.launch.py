from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

server_type = LaunchConfiguration('server_type', default="sub") 
 

def generate_launch_description():
    return LaunchDescription([
        Node(package="corobo_py", executable="crbm_center", arguments=['server_type', server_type])
        ])
