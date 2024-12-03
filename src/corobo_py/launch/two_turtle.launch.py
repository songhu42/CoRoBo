from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
import os 
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_path1 = os.path.join(get_package_share_directory('simple_ros'), 'param', 'turtle_param1.yaml')
    config_path2 = os.path.join(get_package_share_directory('simple_ros'), 'param', 'turtle_param2.yaml')
    config_path3 = os.path.join(get_package_share_directory('simple_ros'), 'param', 'turtle_param3.yaml')

    param_dir1 = LaunchConfiguration('param_dir', default=config_path1)
    param_dir2 = LaunchConfiguration('param_dir', default=config_path2)
    param_dir3 = LaunchConfiguration('param_dir', default=config_path3)

    return LaunchDescription([DeclareLaunchArgument('param_dir', default_value=param_dir1, description='simple parameters'), 
        Node(package="turtlesim", executable="turtlesim_node", parameters=[param_dir1], namespace="Anna01"), 
        Node(package="turtlesim", executable="turtlesim_node", parameters=[param_dir2], namespace="Rola02"), 
        Node(package="turtlesim", executable="turtlesim_node", parameters=[param_dir3], namespace="Sili03"), 
        ExecuteProcess(cmd=[['ros2 service call ', '/Anna01/spawn ', 'turtlesim/srv/Spawn ', '"{x: 2, y: 2, theta: 0.2}"']], shell=True), 
        Node(package='simple_ros', executable='move_turtle_ns', parameters=[param_dir1], namespace="Anna01"),
        Node(package='simple_ros', executable='move_turtle_time_ns', parameters=[param_dir2], namespace="Rola02")
        ])
