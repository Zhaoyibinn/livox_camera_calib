import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    livox_share = get_package_share_directory('livox_ros_driver2')

    start_driver = LaunchConfiguration('start_livox_driver')
    lidar_config = LaunchConfiguration('livox_config_file')
    lidar_topic = LaunchConfiguration('lidar_topic')
    image_topic = LaunchConfiguration('image_topic')
    pcd_file = LaunchConfiguration('pcd_file')
    image_file = LaunchConfiguration('image_file')
    duration = LaunchConfiguration('duration')

    livox_driver = Node(
        condition=IfCondition(start_driver),
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            'xfer_format': 0,
            'multi_topic': 0,
            'data_src': 0,
            'publish_freq': 10.0,
            'output_data_type': 0,
            'frame_id': 'livox_frame',
            'user_config_path': lidar_config,
            'cmdline_input_bd_code': 'livox0000000001',
        }])

    capture_node = Node(
        package='livox_camera_calib',
        executable='capture_calib_sample',
        name='capture_calib_sample',
        output='screen',
        parameters=[{
            'lidar_topic': lidar_topic,
            'image_topic': image_topic,
            'pcd_file': pcd_file,
            'image_file': image_file,
            'duration': ParameterValue(duration, value_type=float),
        }])

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_livox_driver', default_value='true',
            description='Start livox_ros_driver2; set false if it is already running'),
        DeclareLaunchArgument(
            'livox_config_file',
            default_value=os.path.join(livox_share, 'config', 'MID360_config.json')),
        DeclareLaunchArgument('lidar_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('image_topic', default_value='/left_camera/image'),
        DeclareLaunchArgument('duration', default_value='3.0'),
        DeclareLaunchArgument('pcd_file', default_value='/tmp/livox_calib_sample.pcd'),
        DeclareLaunchArgument('image_file', default_value='/tmp/livox_calib_sample.png'),
        livox_driver,
        capture_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=capture_node,
                on_exit=[EmitEvent(event=Shutdown(reason='capture completed'))])),
    ])
