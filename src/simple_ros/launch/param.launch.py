from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import os 
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_path = os.path.join(get_package_share_directory('simple_ros'), 'param', 'simple_param.yaml')

    param_dir = LaunchConfiguration('param_dir', default=config_path)

    print("param.launch.py")
    print(config_path)

    return LaunchDescription([DeclareLaunchArgument('param_dir', default_value=param_dir, description='simple parameters'), 
        Node(package="simple_ros", executable="simple_param", parameters=[param_dir]),
        Node(package="turtlesim", executable="turtlesim_node")
        ])

