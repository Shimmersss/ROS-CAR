"""N10P driver and read-only health monitoring; independent of vehicle control."""
import math
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from perception_bringup.radar_config import driver_spec, mount_args


def start(context):
    config = Path(LaunchConfiguration('radar_config').perform(context))
    c = yaml.safe_load(config.read_text())
    for key in ('model', 'serial_port'):
        value = LaunchConfiguration(key).perform(context)
        if value:
            c[key] = value
    package, executable, parameters, remaps = driver_spec(c)
    if not math.isfinite(c['max_age_s']) or c['max_age_s'] <= 0:
        raise ValueError('max_age_s must be positive and finite')
    actions = [Node(package='perception_bringup', executable='radar_health',
                    name='radar_health', parameters=[{'scan_topic': c['scan_topic'],
                    'frame_id': c['frame_id'], 'max_age_s': c['max_age_s']}], output='screen')]
    if LaunchConfiguration('start_driver').perform(context) == 'true':
        actions.append(Node(package=package, executable=executable, name='roscar_n10p',
                            parameters=[parameters], remappings=remaps, output='screen'))
    transform = mount_args(c)
    if transform:
        actions.append(Node(package='tf2_ros', executable='static_transform_publisher',
                            name='radar_mount', arguments=transform, output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('radar_config', default_value=str(Path(
            get_package_share_directory('perception_bringup')) / 'config/radar.yaml')),
        DeclareLaunchArgument('model', default_value=''),
        DeclareLaunchArgument('serial_port', default_value=''),
        DeclareLaunchArgument('start_driver', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=start),
    ])
