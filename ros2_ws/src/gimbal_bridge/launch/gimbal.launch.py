"""Gimbal bridge (+ optional MCU simulator) and, only for a confirmed mount, its URDF TF.

Standalone; not included by any existing perception or control entry point.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from gimbal_bridge.urdf import build_urdf, load_mount


def nodes(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    mount = load_mount(get('mount_config'))
    mock = get('mock') == 'true'
    allow = get('allow_unconfirmed_mount') == 'true'
    if allow and not mock:
        raise ValueError('allow_unconfirmed_mount is for the simulator only (mock:=true)')
    port = get('mock_link') if mock else get('port')
    if not port:
        raise ValueError('Set port:=/dev/... or mock:=true')
    actions = []
    if mock:
        actions.append(Node(package='gimbal_bridge', executable='mock_mcu', output='screen',
                            arguments=['--link', port, '--mode', get('mock_mode')]))
    actions.append(Node(package='gimbal_bridge', executable='bridge', output='screen', parameters=[{
        'port': port, 'baud': int(get('baud')),
        'base_yaw_rate_source': get('base_yaw_rate_source')}]))
    if mount['confirmed'] or allow:
        actions.append(Node(package='robot_state_publisher', executable='robot_state_publisher',
                            output='screen', parameters=[{'robot_description': build_urdf(mount)}],
                            remappings=[('joint_states', 'gimbal/joint_states')]))
    else:
        actions.append(LogInfo(msg='Gimbal mount UNCONFIRMED: bridge only, no gimbal TF published'))
    return actions


def generate_launch_description():
    share = FindPackageShare('gimbal_bridge')
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value=''),
        DeclareLaunchArgument('baud', default_value='460800'),
        DeclareLaunchArgument('mount_config', default_value=PathJoinSubstitution([share, 'config', 'gimbal_mount.yaml'])),
        DeclareLaunchArgument('mock', default_value='false'),
        DeclareLaunchArgument('mock_mode', default_value='encoder'),
        DeclareLaunchArgument('mock_link', default_value='/tmp/roscar_gimbal_mock'),
        DeclareLaunchArgument('allow_unconfirmed_mount', default_value='false'),
        DeclareLaunchArgument('base_yaw_rate_source', default_value='odom'),
        OpaqueFunction(function=nodes),
    ])
