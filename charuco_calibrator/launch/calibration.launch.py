from launch import LaunchDescription
import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'calibration_type',
            default_value='intrinsic',
            description='Tipo de calibración: intrinsic, extrinsic, hand_eye'
        ),
        DeclareLaunchArgument(
            'images_folder',
            default_value='',
            description='Carpeta con imágenes para calibrar'
        ),
        DeclareLaunchArgument(
            'charuco_rows',
            default_value='5',
            description='Número de filas del tablero Charuco'
        ),
        DeclareLaunchArgument(
            'charuco_cols',
            default_value='7',
            description='Número de columnas del tablero Charuco'
        ),
        
        Node(
            package='charuco_calibrator',
            executable='charuco_intrinsic',
            name='charuco_intrinsic_calibrator',
            parameters=[{
                'images_folder': LaunchConfiguration('images_folder'),
                'charuco_rows': LaunchConfiguration('charuco_rows'),
                'charuco_cols': LaunchConfiguration('charuco_cols'),
            }],
            condition=launch.conditions.IfCondition(
                launch.substitutions.LaunchConfiguration('calibration_type').perform({'calibration_type': 'intrinsic'})
            )
        ),
        
        Node(
            package='charuco_calibrator',
            executable='charuco_hand_eye',
            name='charuco_hand_eye_calibrator',
            parameters=[{
                'images_folder': LaunchConfiguration('images_folder'),
                'charuco_rows': LaunchConfiguration('charuco_rows'),
                'charuco_cols': LaunchConfiguration('charuco_cols'),
            }],
            condition=launch.conditions.IfCondition(
                launch.substitutions.LaunchConfiguration('calibration_type').perform({'calibration_type': 'hand_eye'})
            )
        ),
    ])
