import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    pkg_share = get_package_share_directory('charuco_detector')
    
    # Default config file paths
    default_ros_param_file = os.path.join(pkg_share, 'config', 'ros.yaml')
    default_charuco_param_file = os.path.join(pkg_share, 'yaml', 'charuco.yaml')
    
    # Launch arguments
    ros_param_file_arg = DeclareLaunchArgument(
        'ros_param_file',
        default_value=default_ros_param_file,
        description='Path to ROS parameters YAML file'
    )
    
    charuco_param_file_arg = DeclareLaunchArgument(
        'charuco_param_file',
        default_value=default_charuco_param_file,
        description='Path to ChArUco parameters YAML file'
    )
    
    # Node
    charuco_node = Node(
        package='charuco_detector',
        executable='charuco_detector_node',
        name='charuco_detector',
        output='screen',
        parameters=[
            LaunchConfiguration('ros_param_file'),
            LaunchConfiguration('charuco_param_file')
        ]
    )
    
    return LaunchDescription([
        ros_param_file_arg,
        charuco_param_file_arg,
        charuco_node
    ])
