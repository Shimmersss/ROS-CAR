"""One guard, optional producers; never auto-arms or starts hardware by default."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    get = lambda key: LaunchConfiguration(key).perform(context)
    yes = lambda key: get(key) == 'true'
    config = yaml.safe_load(Path(get('safety_config')).read_text())['/**']['ros__parameters']
    actions = [RegisterEventHandler(OnProcessExit(on_exit=[EmitEvent(event=Shutdown(reason='API module exited'))])),
               Node(package='motion_guard', executable='guard', output='screen',
                    parameters=[get('safety_config'), {'command_mode':'IDLE',
                        'motion_enabled':yes('motion_enabled')}])]
    if yes('with_follower'):
        actions.append(Node(package='astra_body_adapter', executable='person_follower', output='screen',
            parameters=[{'enabled': True, 'expected_source':config['expected_source'],
                         'base_frame':config['base_frame'], 'target_distance_m':float(get('target_distance_m')),
                         'max_linear_mps':float(config['max_linear_mps']),
                         'max_angular_rps':float(config['max_angular_rps'])}]))
    if yes('with_perception'):
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(Path(share('perception_bringup'))/'launch/perception.launch.py')),
            launch_arguments={key:get(key) for key in ('route','model_path','device','depth_registered',
                'color_topic','depth_topic','camera_info_topic','camera_mount_config','yolo_python')}.items()))
    if yes('with_radar'):
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(Path(share('perception_bringup'))/'launch/radar.launch.py')),
            launch_arguments={'radar_config':get('radar_config')}.items()))
    if yes('with_chassis'):
        if not get('serial_port') or not get('car_mode'):
            raise ValueError('Explicit verified serial_port and car_mode required')
        actions.append(Node(package='turn_on_wheeltec_robot', executable='wheeltec_robot_node',
            parameters=[{'usart_port_name':get('serial_port'),'car_mode':get('car_mode'),
                         'serial_baud_rate':115200,'command_timeout_s':.5,'feedback_timeout_s':.5,
                         'max_linear_mps':float(config['max_linear_mps']),
                         'max_angular_rps':float(config['max_angular_rps'])}], output='screen'))
    return actions


def generate_launch_description():
    defaults = dict(motion_enabled='false', with_perception='false', with_follower='false',
        with_radar='false', with_chassis='false', serial_port='', car_mode='', route='yolo',
        model_path='', device='cpu', yolo_python='', depth_registered='false', target_distance_m='1.0',
        color_topic='/camera/color/image_rect', depth_topic='/camera/aligned_depth_to_color/image_raw',
        camera_info_topic='/camera/color/camera_info',
        safety_config=str(Path(share('motion_guard'))/'config/safety.yaml'),
        radar_config=str(Path(share('perception_bringup'))/'config/radar.yaml'),
        camera_mount_config=str(Path(share('perception_bringup'))/'config/camera_mount.yaml'))
    booleans = {'motion_enabled','with_perception','with_follower','with_radar','with_chassis','depth_registered'}
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v,
        **({'choices':['true','false']} if k in booleans else
           {'choices':['yolo','red','astra']} if k == 'route' else {}))
        for k,v in defaults.items()] + [OpaqueFunction(function=start)])
