from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

server_type = LaunchConfiguration('server_type')
print(f"LaunchConfiguration server_type : {server_type}")

server_type = "sub"

def generate_launch_description():
    return LaunchDescription([
        Node(package="corobo_py", executable="crbs_mani", arguments=[{'server_type', server_type}] ),
        Node(package="corobo_py", executable="crbs_server", arguments=[{'server_type', server_type}]) 
        ])
