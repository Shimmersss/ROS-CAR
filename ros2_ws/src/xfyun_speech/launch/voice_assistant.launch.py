import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('xfyun_speech')
    default_config = os.path.join(package_share, 'config', 'voice_assistant.yaml')
    config_argument = DeclareLaunchArgument(
        'config', default_value=default_config,
        description='Voice assistant parameter file.',
    )
    tts_argument = DeclareLaunchArgument(
        'enable_tts', default_value='false',
        description='Start online TTS and ALSA playback.',
    )
    buzzer_argument = DeclareLaunchArgument(
        'enable_buzzer', default_value='false',
        description='Start the hardware buzzer adapter.',
    )
    wake_argument = DeclareLaunchArgument(
        'enable_wake_driver', default_value='false',
        description='Start only the WheelTec microphone serial wake driver.',
    )
    config = LaunchConfiguration('config')
    return LaunchDescription([
        config_argument,
        tts_argument,
        buzzer_argument,
        wake_argument,
        Node(
            package='wheeltec_mic_ros2',
            executable='wheeltec_mic',
            name='wheeltec_mic_wake',
            output='screen',
            parameters=[{
                'usart_port_name': '/dev/wheeltec_mic',
                'serial_baud_rate': 115200,
            }],
            condition=IfCondition(LaunchConfiguration('enable_wake_driver')),
        ),
        Node(
            package='xfyun_speech',
            executable='asr_node',
            name='xfyun_asr',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='voice_command_router',
            executable='router_node',
            name='voice_command_router',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='voice_command_router',
            executable='buzzer_gpio_node',
            name='buzzer_gpio',
            output='screen',
            parameters=[config],
            condition=IfCondition(LaunchConfiguration('enable_buzzer')),
        ),
        Node(
            package='deepseek_ros2',
            executable='chat_node',
            name='deepseek_chat',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='xfyun_speech',
            executable='tts_node',
            name='xfyun_tts',
            output='screen',
            parameters=[config],
            condition=IfCondition(LaunchConfiguration('enable_tts')),
        ),
    ])
