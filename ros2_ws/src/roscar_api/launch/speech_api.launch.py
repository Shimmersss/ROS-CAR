"""Expose /voice/say; reuse running TTS by default, without starting hardware control."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('with_tts', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('with_announcements', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('voice_config', default_value=str(
            Path(get_package_share_directory('offline_voice'))/'config/offline_voice.yaml')),
        DeclareLaunchArgument('playback_device', default_value='plughw:CARD=Device,DEV=0'),
        DeclareLaunchArgument('playback_timeout_sec', default_value='120.0'),
        Node(package='roscar_api', executable='speech', output='screen', parameters=[{
            'playback_timeout_sec': ParameterValue(LaunchConfiguration('playback_timeout_sec'), value_type=float)}]),
        Node(package='roscar_api', executable='phase_announcements', output='screen',
             condition=IfCondition(LaunchConfiguration('with_announcements'))),
        Node(package='offline_voice', executable='tts_node', name='offline_tts', output='screen',
             condition=IfCondition(LaunchConfiguration('with_tts')), parameters=[LaunchConfiguration('voice_config'), {
                 'enable_speech_action': True, 'max_text_chars': 300, 'num_threads': 2,
                 'playback_device': LaunchConfiguration('playback_device')}]),
    ])
