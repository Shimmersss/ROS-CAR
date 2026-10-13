"""Request producer + gate. Never starts sensors or chassis; auto-arm only when explicitly requested."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    path=LaunchConfiguration('safety_config').perform(context)
    c=yaml.safe_load(Path(path).read_text())['/**']['ros__parameters']
    motion=LaunchConfiguration('motion_enabled').perform(context)=='true'
    source=LaunchConfiguration('expected_source').perform(context) or c['expected_source']
    target_frame=LaunchConfiguration('target_frame').perform(context) or c['target_frame']
    auto=LaunchConfiguration('auto_arm').perform(context)=='true'
    recover=LaunchConfiguration('target_auto_recover').perform(context)=='true'
    linear=float(LaunchConfiguration('max_linear_mps').perform(context) or c['max_linear_mps'])
    return [Node(package='astra_body_adapter',executable='person_follower',output='screen',
                 parameters=[{'enabled':True,'expected_source':source,'base_frame':c['base_frame'],
                    'performance_enabled':LaunchConfiguration('performance_enabled').perform(context)=='true',
                    'max_linear_mps':linear,
                    'distance_gain':float(LaunchConfiguration('distance_gain').perform(context)),
                    'bearing_gain':float(LaunchConfiguration('bearing_gain').perform(context)),
                    'max_angular_rps':float(c['max_angular_rps']),
                    'target_distance_m':float(LaunchConfiguration('target_distance_m').perform(context))}]),
            Node(package='motion_guard',executable='guard',output='screen',
                 parameters=[path,{'max_linear_mps':linear,'motion_enabled':motion,'expected_source':source,'target_frame':target_frame,
                    'radar_required':LaunchConfiguration('radar_required').perform(context)=='true',
                    'auto_arm':auto,
                    # An auto-arming follow entry must start in FOLLOW; otherwise the guard starts IDLE.
                    'command_mode':'FOLLOW' if auto or recover else 'IDLE',
                    'target_grace_s':float(LaunchConfiguration('target_grace_s').perform(context)),
                    'target_auto_recover':recover,
                    'emergency_kws_required':LaunchConfiguration('emergency_kws_required').perform(context)=='true',
                    'geometry_confirmed':LaunchConfiguration('geometry_confirmed').perform(context)=='true',
                    'mount_calibrated':LaunchConfiguration('mount_calibrated').perform(context)=='true',
                    'stopping_model_confirmed':LaunchConfiguration('stopping_model_confirmed').perform(context)=='true'}])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('safety_config',default_value=str(Path(
            get_package_share_directory('motion_guard'))/'config/safety.yaml')),
        DeclareLaunchArgument('motion_enabled',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('radar_required',default_value='false',choices=['true','false']),
        # Follow-only: arm by itself once every check passes; STOP returns to IDLE and is not undone.
        DeclareLaunchArgument('auto_arm',default_value='false',choices=['true','false']),
        # Brief target-gap tolerance in FOLLOW (0 = off); see motion_guard/node.py.
        DeclareLaunchArgument('target_grace_s',default_value='0.0'),
        # Re-arm after a target-only FAULT once the target is valid again (Route A behaviour).
        DeclareLaunchArgument('target_auto_recover',default_value='false',choices=['true','false']),
        # Voice emergency-KWS heartbeat; false only when the operator explicitly runs without voice.
        DeclareLaunchArgument('emergency_kws_required',default_value='true',choices=['true','false']),
        DeclareLaunchArgument('geometry_confirmed',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('mount_calibrated',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('stopping_model_confirmed',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('expected_source',default_value=''),
        DeclareLaunchArgument('target_frame',default_value=''),
        DeclareLaunchArgument('performance_enabled',default_value='true',choices=['true','false']),
        DeclareLaunchArgument('target_distance_m',default_value='1.0'),
        DeclareLaunchArgument('max_linear_mps',default_value=''),
        DeclareLaunchArgument('distance_gain',default_value='0.6'),
        DeclareLaunchArgument('bearing_gain',default_value='1.2'),
        OpaqueFunction(function=start)])
