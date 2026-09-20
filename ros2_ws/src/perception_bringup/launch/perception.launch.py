"""Select exactly one perception route; synthetic data remains opt-in."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
    elif route == 'yolo':
        parameters = [{name: ParameterValue(LaunchConfiguration(name).perform(context), value_type=str) for name in
                       ('model_path', 'device', 'color_topic', 'depth_topic', 'camera_info_topic')}]
        parameters[0].update({
            'nms_free': LaunchConfiguration('nms_free').perform(context) == 'true',
            'depth_registered': LaunchConfiguration('depth_registered').perform(context) == 'true',
            'image_size': int(LaunchConfiguration('image_size').perform(context)),
            'sync_slop_s': float(LaunchConfiguration('sync_slop_s').perform(context)),
            'max_age_s': float(LaunchConfiguration('max_age_s').perform(context)),
        })
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
    nodes = [Node(
        **node_options, package=package, executable=executable,
        namespace='perception', output='screen',
        parameters=parameters,
    )]
    if route == 'yolo':
        nodes.append(Node(package='yolo_person_tracker', executable='target_transform',
                          namespace='perception', output='screen',
                          parameters=[LaunchConfiguration('camera_mount_config').perform(context)]))
    return nodes


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
        DeclareLaunchArgument('akimbo_hand_above_base_min_mm', default_value='50.0'),
        DeclareLaunchArgument('akimbo_hand_shoulder_max_dx_mm', default_value='100.0'),
        DeclareLaunchArgument('akimbo_shoulder_above_hand_min_mm', default_value='50.0'),
        DeclareLaunchArgument('akimbo_window_frames', default_value='1'),
        DeclareLaunchArgument('akimbo_min_votes', default_value='1'),
        DeclareLaunchArgument('camera_mount_config', default_value=PathJoinSubstitution([
            FindPackageShare('perception_bringup'), 'config', 'camera_mount.yaml'])),
        DeclareLaunchArgument('yolo_python', default_value=''),
        DeclareLaunchArgument('model_path', default_value=''),
        DeclareLaunchArgument('device', default_value='cpu'),
        DeclareLaunchArgument('nms_free', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('image_size', default_value='640'),
        DeclareLaunchArgument('depth_registered', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('color_topic', default_value='/camera/color/image_rect'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/aligned_depth_to_color/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/color/camera_info'),
        DeclareLaunchArgument('sync_slop_s', default_value='0.06'),
        DeclareLaunchArgument('max_age_s', default_value='0.5'),
        OpaqueFunction(function=launch_route),
        OpaqueFunction(function=launch_bridge),
    ])
