"""Select exactly one perception route; synthetic data remains opt-in."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_route(context):
    route = LaunchConfiguration('route').perform(context)
    routes = {
        'astra': ('astra_body_adapter', 'bodylist_adapter'),
        'yolo': ('yolo_person_tracker', 'tracker'),
        'demo': ('perception_bringup', 'demo'),
    }
    package, executable = routes[route]
    config = os.path.join(get_package_share_directory('perception_bringup'), 'config', 'demo.yaml')
    parameters = []
    if route == 'demo':
        parameters = [config]
    elif route == 'astra':
        parameters = [{
            'akimbo_hand_above_base_min_mm': float(LaunchConfiguration(
                'akimbo_hand_above_base_min_mm').perform(context)),
            'akimbo_hand_shoulder_max_dx_mm': float(LaunchConfiguration(
                'akimbo_hand_shoulder_max_dx_mm').perform(context)),
            'akimbo_shoulder_above_hand_min_mm': float(LaunchConfiguration(
                'akimbo_shoulder_above_hand_min_mm').perform(context)),
            'akimbo_window_frames': int(LaunchConfiguration(
                'akimbo_window_frames').perform(context)),
            'akimbo_min_votes': int(LaunchConfiguration(
                'akimbo_min_votes').perform(context)),
        }]
    return [Node(
        package=package, executable=executable,
        namespace='perception', output='screen',
        parameters=parameters,
    )]


def launch_bridge(context):
    if LaunchConfiguration('with_foxglove').perform(context) != 'true':
        return []
    # Resolve only when enabled; the core workspace does not require the bridge.
    bridge = os.path.join(get_package_share_directory('foxglove_bridge'),
                          'launch', 'foxglove_bridge_launch.xml')
    return [IncludeLaunchDescription(AnyLaunchDescriptionSource(bridge))]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('route', default_value='yolo', choices=['astra', 'yolo', 'demo']),
        DeclareLaunchArgument('with_foxglove', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('akimbo_hand_above_base_min_mm', default_value='20.0'),
        DeclareLaunchArgument('akimbo_hand_shoulder_max_dx_mm', default_value='160.0'),
        DeclareLaunchArgument('akimbo_shoulder_above_hand_min_mm', default_value='20.0'),
        DeclareLaunchArgument('akimbo_window_frames', default_value='10'),
        DeclareLaunchArgument('akimbo_min_votes', default_value='3'),
        OpaqueFunction(function=launch_route),
        OpaqueFunction(function=launch_bridge),
    ])
