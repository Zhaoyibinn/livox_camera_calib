import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('livox_camera_calib')
    params_file = LaunchConfiguration('params_file')
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(package_share, 'config', 'calib.yaml')),
        Node(
            package='livox_camera_calib',
            executable='lidar_camera_calib',
            name='lidar_camera_calib',
            parameters=[params_file],
            output='screen'),
    ])
