import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('offline_voice')
    config = LaunchConfiguration('config')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value=os.path.join(share, 'config', 'offline_voice.yaml')),
        DeclareLaunchArgument('enable_tts', default_value='true'),
        DeclareLaunchArgument('use_xfyun_asr', default_value='false'),
        DeclareLaunchArgument('continuous_asr', default_value='true'),
        DeclareLaunchArgument('legacy_listen_polling',
                              default_value=LaunchConfiguration('use_xfyun_asr')),
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
            package='offline_voice', executable='continuous_asr_node',
            name='offline_asr', output='screen', parameters=[config],
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('use_xfyun_asr'), "' == 'false' and '",
                LaunchConfiguration('continuous_asr'), "' == 'true'",
            ])),
        ),
        Node(
            package='offline_voice', executable='asr_node',
            name='offline_asr', output='screen',
            parameters=[config, {'wake_text_topic': ''}],
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('use_xfyun_asr'), "' == 'false' and '",
                LaunchConfiguration('continuous_asr'), "' == 'false'",
            ])),
        ),
        Node(
            package='xfyun_speech', executable='asr_node',
            # Retain the parameter namespace and existing topic/service contract.
            name='offline_asr', output='screen',
            parameters=[config, {'speech_timeout_sec': 5.0, 'speech_onset_ms': 160,
                                 'wake_text_topic': ''}],
            condition=IfCondition(LaunchConfiguration('use_xfyun_asr')),
        ),
        Node(
            package='voice_command_router', executable='router_node',
            name='voice_command_router', output='screen',
            parameters=[config, {'enabled': LaunchConfiguration('voice_control_enabled'),
                                 'legacy_listen_polling': LaunchConfiguration(
                                     'legacy_listen_polling')}],
        ),
        Node(
            package='offline_voice', executable='tts_node',
            name='offline_tts', output='screen', parameters=[config],
            condition=IfCondition(LaunchConfiguration('enable_tts')),
        ),
    ])
