import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    pkg_share = get_package_share_directory('charuco_detector')
    
    # Launch arguments
    eye_in_hand_arg = DeclareLaunchArgument(
        'eye_in_hand_mode',
        default_value='true',
        description='Eye-in-hand (true) or eye-to-hand (false) calibration'
    )
    
    base_link_arg = DeclareLaunchArgument(
        'base_link',
        default_value='base_link',
        description='Robot base frame'
    )
    
    tip_link_arg = DeclareLaunchArgument(
        'tip_link',
        default_value='tool0',
        description='Robot tool frame'
    )
    
    customize_arg = DeclareLaunchArgument(
        'customize',
        default_value='false',
        description='Use custom calibration file'
    )
    
    filename_arg = DeclareLaunchArgument(
        'filename',
        default_value='',
        description='Custom calibration filename'
    )
    
    # Include charuco detector
    charuco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'charuco_detector.launch.py')
        )
    )
    
    # Hand-eye transformation node
    hand_eye_node = Node(
        package='charuco_detector',
        executable='hand_eye_trans.py',
        name='hand_eye_trans',
        output='screen',
        parameters=[{
            'eye_in_hand_mode': LaunchConfiguration('eye_in_hand_mode'),
            'base_link': LaunchConfiguration('base_link'),
            'tip_link': LaunchConfiguration('tip_link'),
            'customize': LaunchConfiguration('customize'),
            'filename': LaunchConfiguration('filename')
        }]
    )
    
    return LaunchDescription([
        eye_in_hand_arg,
        base_link_arg,
        tip_link_arg,
        customize_arg,
        filename_arg,
        charuco_launch,
        hand_eye_node
    ])
