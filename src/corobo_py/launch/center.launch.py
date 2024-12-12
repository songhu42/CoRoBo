from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import LaunchConfiguration, ExecuteProcess
from launch.actions import DeclareLaunchArgument

server_type = LaunchConfiguration('server_type', default="sub") 
server_type_arg = DeclareLaunchArgument(
        'server_type',
        default_value='sub', description='Whether main or sub' )

change_server_type_arg = ExecuteProcess(
        cmd=[[
            'ros2 param set ',
            'corobo_py',
            '/crbm_center server_type ',
            'sub'
        ]],
        shell=True
    )

def generate_launch_description():
    return LaunchDescription([
        Node(package="corobo_py", executable="crbm_center", arguments=[{'server_type', server_type}] )
        ])
