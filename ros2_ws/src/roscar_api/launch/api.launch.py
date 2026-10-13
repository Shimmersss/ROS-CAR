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
    control = {'command_mode': 'IDLE', 'auto_arm': False,
               'motion_enabled': yes('motion_enabled'), 'radar_required': yes('radar_required'),
               'expected_source': get('expected_source'), 'target_frame': get('target_frame'),
               'target_topic': get('target_topic')}
    if yes('with_voice') and yes('speech_with_tts'):
        raise ValueError('with_voice already owns the single TTS; speech_with_tts must stay false')
    actions = [RegisterEventHandler(OnProcessExit(on_exit=[EmitEvent(event=Shutdown(reason='API module exited'))])),
               Node(package='motion_guard', executable='guard', output='screen',
                    parameters=[get('safety_config'), control])]
    if yes('with_follower'):
        actions.append(Node(package='astra_body_adapter', executable='person_follower', output='screen',
            remappings=[('/perception/target_state', get('target_topic'))],
            parameters=[{'enabled': True, 'expected_source':get('expected_source'),
                         'base_frame':config['base_frame'], 'target_distance_m':float(get('target_distance_m')),
                         'max_linear_mps':float(config['max_linear_mps']),
                         'max_angular_rps':float(config['max_angular_rps'])}]))
    if yes('with_perception'):
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(Path(share('perception_bringup'))/'launch/perception.launch.py')),
            launch_arguments={key:get(key) for key in ('route','model_path','device','depth_registered',
                'color_topic','depth_topic','camera_info_topic','camera_mount_config','yolo_python',
                'pose_config','reid_config','reid_enabled','reid_lock_enabled','reid_model_path',
                'sot_enabled','sot_model_path','reid_reacquire_enabled','auto_lock_single',
                'owner_lost_timeout_s','registration_calibration_file','registration_expected_serial')}.items()))
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
    def include(package, filename, arguments):
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(Path(share(package))/'launch'/filename)), launch_arguments=arguments.items()))
    if yes('with_voice'):
        include('offline_voice', 'offline_voice.launch.py', {
            'config': get('voice_config'), 'enable_speech_action': 'true',
            'enable_wake_driver': get('enable_wake_driver'), 'use_xfyun_asr': get('use_xfyun_asr'),
            'execution_modes_config': get('modes_config') if yes('with_tongue') else ''})
    if yes('with_speech') or yes('with_voice'):
        include('roscar_api', 'speech_api.launch.py', {'with_tts': get('speech_with_tts'),
            'voice_config': get('voice_config'), 'with_announcements': get('with_announcements')})
    if yes('with_tongue'):
        include('tongue_mode', 'tongue_mode.launch.py', {
            'with_guard': 'false', 'with_voice': 'false', 'with_router': 'false',
            'config': get('tongue_config'), 'catalog': get('modes_config'),
            'safety_profile': get('tongue_safety_profile'), 'server_url': get('tongue_server_url'),
            'camera_topic': get('tongue_camera_topic'),
            'camera_profile': get('tongue_camera_profile') or (
                'gemini_rgb_tongue_v1' if get('route') == 'yolo_pose' else 'astra_rgb_tongue_v1')})
    if yes('with_gimbal'):
        include('gimbal_bridge', 'gimbal.launch.py', {'port': get('gimbal_port'),
            'mount_config': get('gimbal_mount_config'), 'mock': get('gimbal_mock')})
    return actions


def generate_launch_description():
    defaults = dict(motion_enabled='false', with_perception='false', with_follower='false',
        with_radar='false', with_chassis='false', serial_port='', car_mode='', route='yolo',
        model_path='', device='cpu', yolo_python='', depth_registered='false', target_distance_m='1.0',
        radar_required='false', expected_source='yolo', target_frame='camera_color_optical_frame',
        target_topic='/perception/target_state', auto_lock_single='false', owner_lost_timeout_s='10.0',
        reid_enabled='false', reid_lock_enabled='false', reid_model_path='',
        sot_enabled='false', sot_model_path='', reid_reacquire_enabled='false',
        registration_calibration_file='', registration_expected_serial='',
        with_voice='false', with_speech='false', speech_with_tts='false', with_announcements='true',
        enable_wake_driver='true', use_xfyun_asr='false', with_tongue='false',
        tongue_safety_profile='robot', tongue_server_url='http://172.19.128.171:8096',
        tongue_camera_topic='/camera/color/image_raw', tongue_camera_profile='',
        with_gimbal='false', gimbal_port='', gimbal_mock='false',
        color_topic='/camera/color/image_rect', depth_topic='/camera/aligned_depth_to_color/image_raw',
        camera_info_topic='/camera/color/camera_info',
        voice_config=str(Path(share('offline_voice'))/'config/offline_voice.yaml'),
        tongue_config=str(Path(share('tongue_mode'))/'config/tongue.yaml'),
        modes_config=str(Path(share('execution_mode_manager'))/'config/modes.yaml'),
        gimbal_mount_config=str(Path(share('gimbal_bridge'))/'config/gimbal_mount.yaml'),
        pose_config=str(Path(share('perception_bringup'))/'config/pose.yaml'),
        reid_config=str(Path(share('perception_bringup'))/'config/reid.yaml'),
        safety_config=str(Path(share('motion_guard'))/'config/safety.yaml'),
        radar_config=str(Path(share('perception_bringup'))/'config/radar.yaml'),
        camera_mount_config=str(Path(share('perception_bringup'))/'config/camera_mount.yaml'))
    booleans = {key for key, value in defaults.items() if value in ('true', 'false')}
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v,
        **({'choices':['true','false']} if k in booleans else
           {'choices':['yolo','yolo_pose','red','astra']} if k == 'route' else
           {'choices':['robot','bench']} if k == 'tongue_safety_profile' else {}))
        for k,v in defaults.items()] + [OpaqueFunction(function=start)])
