"""Red Route A: external registered RGB-D; chassis and motion are opt-in."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    chassis = get('with_chassis') == 'true'
    motion = get('motion_enabled') == 'true'
    if motion and not chassis:
        raise ValueError('motion_enabled requires with_chassis')
    if chassis and (not get('car_mode') or not get('serial_port')):
        raise ValueError('Explicit verified car_mode and serial_port required for chassis')
    arguments = {k: get(k) for k in (
        'performance_enabled', 'depth_registered', 'color_topic', 'depth_topic', 'camera_info_topic',
        'sync_slop_s', 'max_age_s', 'hue_low_max', 'hue_high_min',
        'saturation_min', 'value_min', 'min_area_fraction', 'confirm_frames', 'lost_timeout_s')}
    arguments.update(route='red', with_foxglove='false')
    perception = os.path.join(get_package_share_directory('perception_bringup'),
                              'launch', 'perception.launch.py')
    actions = [RegisterEventHandler(OnProcessExit(
        on_exit=[EmitEvent(event=Shutdown(reason='Route A process exited'))])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(perception),
                                        launch_arguments=arguments.items())]
    if chassis:
        actions += [
            Node(package='turn_on_wheeltec_robot', executable='wheeltec_robot_node',
                 output='screen', parameters=[dict(
                     usart_port_name=get('serial_port'), serial_baud_rate=int(get('serial_baud_rate')),
                     car_mode=get('car_mode'), command_timeout_s=.5, feedback_timeout_s=.5,
                     max_linear_mps=.15, max_angular_rps=.5)]),
            Node(package='astra_body_adapter', executable='person_follower',
                 output='screen', parameters=[dict(enabled=motion, expected_source='red_object',
                                                 performance_enabled=get('performance_enabled') == 'true')]),
        ]
    return actions


def generate_launch_description():
    defaults = dict(performance_enabled='true', with_chassis='false', motion_enabled='false', car_mode='', serial_port='',
                    serial_baud_rate='115200', depth_registered='false',
                    color_topic='/camera/color/image_rect',
                    depth_topic='/camera/aligned_depth_to_color/image_raw',
                    camera_info_topic='/camera/color/camera_info', sync_slop_s='0.06', max_age_s='0.5',
                    hue_low_max='10', hue_high_min='170', saturation_min='100', value_min='70',
                    min_area_fraction='0.001', confirm_frames='3', lost_timeout_s='1.0')
    return LaunchDescription([
        DeclareLaunchArgument(k, default_value=v,
                              **({'choices': ['true', 'false']} if k in
                                 ('with_chassis', 'motion_enabled', 'depth_registered', 'performance_enabled') else {}))
        for k, v in defaults.items()] + [OpaqueFunction(function=start)])
