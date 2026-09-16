"""Select exactly one perception route; synthetic data remains opt-in."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_route(context):
    route = LaunchConfiguration('route').perform(context)
    routes = {
        'astra': ('astra_body_adapter', 'bodylist_adapter'),
        'red': ('red_object_tracker', 'tracker'),
        'yolo': ('yolo_person_tracker', 'tracker'),
        'demo': ('perception_bringup', 'demo'),
    }
    package, executable = routes[route]
    config = os.path.join(get_package_share_directory('perception_bringup'), 'config', 'demo.yaml')
    parameters = []
    if route == 'demo':
        parameters = [config]
    elif route in ('yolo', 'red'):
        parameters = [{name: ParameterValue(LaunchConfiguration(name).perform(context), value_type=str) for name in
                       ('model_path', 'device', 'color_topic', 'depth_topic', 'camera_info_topic')}]
        parameters[0].update({
            'depth_registered': LaunchConfiguration('depth_registered').perform(context) == 'true',
            'image_size': int(LaunchConfiguration('image_size').perform(context)),
            'sync_slop_s': float(LaunchConfiguration('sync_slop_s').perform(context)),
            'max_age_s': float(LaunchConfiguration('max_age_s').perform(context)),
        })
        if route == 'red':
            parameters[0]['performance_enabled'] = LaunchConfiguration('performance_enabled').perform(context) == 'true'
            for name in ('model_path', 'device', 'image_size'):
                parameters[0].pop(name)
            for name in ('hue_low_max', 'hue_high_min', 'saturation_min', 'value_min', 'confirm_frames'):
                parameters[0][name] = int(LaunchConfiguration(name).perform(context))
            for name in ('min_area_fraction', 'lost_timeout_s'):
                parameters[0][name] = float(LaunchConfiguration(name).perform(context))
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
    node_options = {}
    if route == 'yolo':
        python = LaunchConfiguration('yolo_python').perform(context)
        if python:
            # The installed console script may have a system-Python shebang.
            import shlex
            node_options['prefix'] = [shlex.quote(python)]
    return [Node(
        **node_options, package=package, executable=executable,
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
        DeclareLaunchArgument('route', default_value='yolo', choices=['astra', 'yolo', 'red', 'demo']),
        DeclareLaunchArgument('performance_enabled', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('with_foxglove', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('akimbo_hand_above_base_min_mm', default_value='50.0'),
        DeclareLaunchArgument('akimbo_hand_shoulder_max_dx_mm', default_value='100.0'),
        DeclareLaunchArgument('akimbo_shoulder_above_hand_min_mm', default_value='50.0'),
        DeclareLaunchArgument('akimbo_window_frames', default_value='1'),
        DeclareLaunchArgument('akimbo_min_votes', default_value='1'),
        DeclareLaunchArgument('yolo_python', default_value=''),
        DeclareLaunchArgument('model_path', default_value=''),
        DeclareLaunchArgument('device', default_value='cpu'),
        DeclareLaunchArgument('image_size', default_value='640'),
        DeclareLaunchArgument('depth_registered', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('color_topic', default_value='/camera/color/image_rect'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/aligned_depth_to_color/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/color/camera_info'),
        DeclareLaunchArgument('sync_slop_s', default_value='0.06'),
        DeclareLaunchArgument('max_age_s', default_value='0.5'),
        DeclareLaunchArgument('hue_low_max', default_value='10'),
        DeclareLaunchArgument('hue_high_min', default_value='170'),
        DeclareLaunchArgument('saturation_min', default_value='100'),
        DeclareLaunchArgument('value_min', default_value='70'),
        DeclareLaunchArgument('min_area_fraction', default_value='0.001'),
        DeclareLaunchArgument('confirm_frames', default_value='3'),
        DeclareLaunchArgument('lost_timeout_s', default_value='1.0'),
        OpaqueFunction(function=launch_route),
        OpaqueFunction(function=launch_bridge),
    ])
