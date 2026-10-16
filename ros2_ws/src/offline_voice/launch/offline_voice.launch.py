import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('offline_voice')
    config = LaunchConfiguration('config')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value=os.path.join(share, 'config', 'offline_voice.yaml')),
        DeclareLaunchArgument('enable_tts', default_value='true'),
        DeclareLaunchArgument('with_motion_adapter', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('use_xfyun_asr', default_value='false'),
        DeclareLaunchArgument('enable_wake_driver', default_value='true'),
        DeclareLaunchArgument('voice_control_enabled', default_value='true'),
        DeclareLaunchArgument('voice_estop_enabled', default_value='true', choices=['true','false']),
        DeclareLaunchArgument('execution_modes_config', default_value=''),
        DeclareLaunchArgument('enable_speech_action', default_value='false'),
        Node(
            package='wheeltec_mic_ros2', executable='wheeltec_mic',
            name='wheeltec_mic_wake', output='screen',
            parameters=[{'usart_port_name': '/dev/wheeltec_mic',
                         'serial_baud_rate': 115200}],
            condition=IfCondition(LaunchConfiguration('enable_wake_driver')),
        ),
        Node(package='offline_voice', executable='audio_node',
             name='voice_audio_capture', output='screen', parameters=[config]),
        Node(package='offline_voice', executable='emergency_node',
             name='emergency_kws', output='screen', parameters=[config],
             condition=IfCondition(LaunchConfiguration('voice_estop_enabled'))),
        Node(package='voice_command_router', executable='motion_adapter',
             name='chassis_motion_adapter', output='screen', parameters=[config],
             condition=IfCondition(LaunchConfiguration('with_motion_adapter'))),
        Node(
            package='offline_voice', executable='asr_node',
            name='offline_asr', output='screen', parameters=[config, {'audio_source': 'topic'}],
            condition=UnlessCondition(LaunchConfiguration('use_xfyun_asr')),
        ),
        Node(
            package='xfyun_speech', executable='asr_node',
            # Retain the parameter namespace and existing topic/service contract.
            name='offline_asr', output='screen',
            parameters=[config, {'audio_source': 'topic', 'speech_timeout_sec': 5.0, 'speech_onset_ms': 160}],
            condition=IfCondition(LaunchConfiguration('use_xfyun_asr')),
        ),
        Node(
            package='voice_command_router', executable='router_node',
            name='voice_command_router', output='screen',
            parameters=[config, {'enabled': LaunchConfiguration('voice_control_enabled'),
                                'voice_estop_enabled': ParameterValue(LaunchConfiguration('voice_estop_enabled'), value_type=bool),
                                'execution_modes_config': LaunchConfiguration('execution_modes_config')}],
        ),
        Node(
            package='offline_voice', executable='tts_node',
            name='offline_tts', output='screen', parameters=[config, {
                'enable_speech_action': ParameterValue(LaunchConfiguration('enable_speech_action'), value_type=bool)}],
            condition=IfCondition(LaunchConfiguration('enable_tts')),
        ),
    ])
