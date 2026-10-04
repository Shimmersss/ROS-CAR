"""Defaults to module-only integration; hardware and moving profiles remain external."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    get = lambda key: LaunchConfiguration(key).perform(context)
    catalog = get('catalog')
    profile = get('safety_profile')
    with_guard, with_voice, with_router = (get(k) == 'true' for k in ('with_guard', 'with_voice', 'with_router'))
    if with_voice and with_router:
        raise RuntimeError('with_voice already starts the single voice router')
    if with_guard and profile != 'bench':
        raise RuntimeError('This launch only creates a motion-disabled bench guard; use the existing robot guard')
    nodes = [Node(package='execution_mode_manager', executable='manager_node', output='screen',
                  parameters=[{'catalog': catalog}]),
             Node(package='tongue_mode', executable='mode_node', output='screen',
                  parameters=[get('config'), {'safety_profile': profile, 'server_url': get('server_url')}])]
    if with_guard:
        nodes.append(Node(package='motion_guard', executable='guard', output='screen',
                          parameters=[{'motion_enabled': False, 'command_mode': 'IDLE', 'auto_arm': False}]))
    if with_router:
        nodes.append(Node(package='voice_command_router', executable='router_node', output='screen',
                          parameters=[{'execution_modes_config': catalog}]))
    if with_voice:
        path = Path(get_package_share_directory('offline_voice'))/'launch/offline_voice.launch.py'
        nodes.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
                     launch_arguments={'execution_modes_config': catalog, 'enable_speech_action': 'true'}.items()))
    return nodes


def generate_launch_description():
    tongue = Path(get_package_share_directory('tongue_mode'))
    manager = Path(get_package_share_directory('execution_mode_manager'))
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(tongue/'config/tongue.yaml')),
        DeclareLaunchArgument('catalog', default_value=str(manager/'config/modes.yaml')),
        DeclareLaunchArgument('server_url', default_value='http://172.19.128.171:8096'),
        DeclareLaunchArgument('safety_profile', default_value='robot', choices=['robot', 'bench']),
        DeclareLaunchArgument('with_guard', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('with_voice', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('with_router', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=start)])
