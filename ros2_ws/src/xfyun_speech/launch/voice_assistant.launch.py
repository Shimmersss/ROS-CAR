import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TTS_TUNING = ('speed', 'pitch', 'volume')


def tts_node(context):
    """Overlay explicit launch arguments on top of the shared parameter file.

    Only non-empty arguments override, so voice_assistant.yaml stays the single
    default source and `ros2 launch ... voice_name:=x` is enough to switch.
    """
    config = LaunchConfiguration('config').perform(context)
    overrides = {}

    voice_name = LaunchConfiguration('voice_name').perform(context).strip()
    if voice_name:
        overrides['voice_name'] = voice_name

    for name in TTS_TUNING:
        raw = LaunchConfiguration(name).perform(context).strip()
        if not raw:
            continue
        try:
            value = int(raw)
        except ValueError as exc:
            raise ValueError(f'{name} 必须是 0-100 的整数, 实际为 {raw!r}') from exc
        if not 0 <= value <= 100:
            raise ValueError(f'{name} 必须在 0-100 之间, 实际为 {value}')
        overrides[name] = value

    parameters = [config, overrides] if overrides else [config]
    return [Node(
        package='xfyun_speech',
        executable='tts_node',
        name='xfyun_tts',
        output='screen',
        parameters=parameters,
        condition=IfCondition(LaunchConfiguration('enable_tts')),
    )]


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
    voice_argument = DeclareLaunchArgument(
        'voice_name', default_value='',
        description='iFLYTEK vcn; empty keeps voice_assistant.yaml.',
    )
    tuning_arguments = [
        DeclareLaunchArgument(
            name, default_value='',
            description=f'{name} 0-100; empty keeps voice_assistant.yaml.',
        )
        for name in TTS_TUNING
    ]
    config = LaunchConfiguration('config')
    return LaunchDescription([
        config_argument,
        tts_argument,
        buzzer_argument,
        wake_argument,
        voice_argument,
        *tuning_arguments,
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
        OpaqueFunction(function=tts_node),
    ])
