from pathlib import Path
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def nodes(context):
    value=lambda key:LaunchConfiguration(key).perform(context)
    fake=value('fake_tts')=='true';synthetic=value('synthetic_camera')=='true'
    if (fake or synthetic) and (value('safety_profile')!='bench' or os.environ.get('ROS_LOCALHOST_ONLY')!='1' or os.environ.get('ROS_DOMAIN_ID') not in ('187','189','190')):
        raise RuntimeError('Fake resources require bench + isolated domain 187/189/190 and localhost')
    pipeline=value('pipeline_mode')
    if pipeline not in ('upload','infer') or pipeline=='infer' and not value('inference_url'):
        raise RuntimeError('invalid_pipeline_mode_or_missing_inference_url')
    common={'pipeline_mode':pipeline,'cloud_url':value('cloud_url'),'robot_id':value('robot_id'),'token_file':value('token_file')}
    result=[Node(package='capture_task',executable='executor_node',output='screen',parameters=[common,{
        'inference_url':value('inference_url'),'inference_token_file':value('inference_token_file'),
        'return_original_image':ParameterValue(LaunchConfiguration('return_original_image'),value_type=bool),
        'catalog':value('catalog'),'camera_topic':value('camera_topic'),'camera_profile':value('camera_profile'),
        'safety_profile':value('safety_profile'),'storage_dir':value('storage_dir'),
        'save_images':ParameterValue(LaunchConfiguration('save_images'),value_type=bool)}])]
    if value('with_bridge')=='true':
        result.append(Node(package='capture_task',executable='bridge_node',output='screen',parameters=[common,{'return_original_image':ParameterValue(LaunchConfiguration('return_original_image'),value_type=bool),'catalog':value('catalog'),'poll_wait_sec':ParameterValue(LaunchConfiguration('poll_wait_sec'),value_type=int)}]))
    if value('with_ros_api')=='true':
        result.append(Node(package='capture_task',executable='api_node',output='screen',parameters=[{'catalog':value('catalog'),'pipeline_mode':pipeline}]))
    if fake:
        result.append(Node(package='capture_task',executable='fake_tts_node',output='screen',parameters=[{'test_only':True}]))
    if synthetic:
        result.append(Node(package='capture_task',executable='synthetic_camera_node',output='screen',parameters=[{'test_only':True}]))
    return result


def generate_launch_description():
    share=Path(get_package_share_directory('capture_task'))
    defaults=dict(pipeline_mode=os.environ.get('CAPTURE_PIPELINE_MODE','upload'),inference_url=os.environ.get('CAPTURE_INFERENCE_URL',''),
                  inference_token_file=os.environ.get('CAPTURE_INFERENCE_TOKEN_FILE',''),return_original_image='false',poll_wait_sec=os.environ.get('CAPTURE_POLL_WAIT_SEC','1'),catalog=str(share/'config/tasks.yaml'),cloud_url=os.environ.get('CAPTURE_CLOUD_URL','http://127.0.0.1:8090'),
                  robot_id=os.environ.get('CAPTURE_ROBOT_ID','roscar-01'),token_file=os.environ.get('CAPTURE_TOKEN_FILE',''),
                  camera_topic='/camera/color/image_raw',camera_profile='raw_rgb_v1',safety_profile='bench',
                  storage_dir='artifacts/capture-task',save_images='false',with_bridge='true',with_ros_api='false',fake_tts='false',synthetic_camera='false')
    return LaunchDescription([DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()]+[OpaqueFunction(function=nodes)])
