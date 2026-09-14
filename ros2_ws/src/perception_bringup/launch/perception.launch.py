"""Select exactly one route. Defaults to an unready scaffold, never synthetic data."""
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
        'astra': ('astra_body_adapter', 'adapter'),
        'yolo': ('yolo_person_tracker', 'tracker'),
        'demo': ('perception_bringup', 'demo'),
    }
    package, executable = routes[route]
    config = os.path.join(get_package_share_directory('perception_bringup'), 'config', 'demo.yaml')
    return [Node(
        package=package, executable=executable,
        namespace='perception', output='screen',
        parameters=[config] if route == 'demo' else [],
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
        OpaqueFunction(function=launch_route),
        OpaqueFunction(function=launch_bridge),
    ])
