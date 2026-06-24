from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='livox_camera_calib',
            executable='bag_to_pcd',
            name='bag_to_pcd',
            parameters=[{
                'bag_file': '',
                'lidar_topic': '/livox/lidar',
                'pcd_file': '',
                'is_custom_msg': False,
            }],
            output='screen'),
    ])
