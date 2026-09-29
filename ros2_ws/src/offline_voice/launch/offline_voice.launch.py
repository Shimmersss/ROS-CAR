import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('offline_voice')
    config = LaunchConfiguration('config')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value=os.path.join(share, 'config', 'offline_voice.yaml')),
        DeclareLaunchArgument('enable_tts', default_value='true'),
        DeclareLaunchArgument('enable_wake_driver', default_value='true'),
        DeclareLaunchArgument('voice_control_enabled', default_value='true'),
        Node(
            package='wheeltec_mic_ros2', executable='wheeltec_mic',
            name='wheeltec_mic_wake', output='screen',
            parameters=[{'usart_port_name': '/dev/wheeltec_mic',
                         'serial_baud_rate': 115200}],
            condition=IfCondition(LaunchConfiguration('enable_wake_driver')),
        ),
        Node(
            package='offline_voice', executable='asr_node',
            name='offline_asr', output='screen', parameters=[config],
        ),
        Node(
            package='voice_command_router', executable='router_node',
            name='voice_command_router', output='screen',
            parameters=[config, {'enabled': LaunchConfiguration('voice_control_enabled')}],
        ),
        Node(
            package='offline_voice', executable='chat_node',
            name='offline_chat', output='screen', parameters=[config],
        ),
        Node(
            package='offline_voice', executable='tts_node',
            name='offline_tts', output='screen', parameters=[config],
            condition=IfCondition(LaunchConfiguration('enable_tts')),
        ),
    ])
