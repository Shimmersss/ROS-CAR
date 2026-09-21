"""Local navigation stack; never opens chassis ports or automatically arms motion."""
from pathlib import Path
import math
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    get=lambda k:LaunchConfiguration(k).perform(context)
    root=Path(get_package_share_directory('navigation_bringup'))
    safety=yaml.safe_load(Path(get('safety_config')).read_text())['/**']['ros__parameters']
    from motion_guard.safety import SafetyConfig
    SafetyConfig(**{k:safety[k] for k in SafetyConfig.__dataclass_fields__})
    follow=yaml.safe_load(Path(get('follow_config')).read_text())['navigation_follow_goal']['ros__parameters']
    if follow.get('base_frame','base_footprint')!='base_footprint':
        raise ValueError('Navigation base_frame must match chassis base_footprint')
    nav=yaml.safe_load((root/'config/nav2.yaml').read_text())
    # Match the guard's conservative stopping disk, including scan sampling margin.
    radius=(math.hypot(safety['length_m'],safety['width_m'])/2+safety['margin_m']+
            safety['max_linear_mps']*safety['reaction_s']+
            safety['max_linear_mps']**2/(2*safety['deceleration_mps2'])+
            safety['sampling_margin_m']+.01+math.radians(2)*(
                safety['max_linear_mps']*safety['reaction_s']+
                safety['max_linear_mps']**2/(2*safety['deceleration_mps2'])))
    if not math.isfinite(radius) or radius<=0:raise ValueError('Invalid safety geometry')
    for key in ('local_costmap','global_costmap'):
        c=nav[key][key]['ros__parameters'];c['robot_radius']=radius
        c['inflation_layer']['inflation_radius']=radius+.15
    f=nav['controller_server']['ros__parameters']['FollowPath']
    f.update(max_vel_x=float(safety['max_linear_mps']),max_speed_xy=float(safety['max_linear_mps']),
             max_vel_theta=float(safety['max_angular_rps']),
             acc_lim_x=float(safety['deceleration_mps2']),decel_lim_x=-float(safety['deceleration_mps2']))
    nav['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml']=str(root/'behavior_trees/follow.xml')
    nav['bt_navigator']['ros__parameters']['default_nav_through_poses_bt_xml']=str(root/'behavior_trees/through_poses.xml')
    # ROS parameter files preserve nested costmap namespaces.
    from launch_ros.parameter_descriptions import ParameterFile
    import tempfile
    import atexit
    out=tempfile.NamedTemporaryFile(mode='w',prefix='roscar-nav-',suffix='.yaml',delete=False)
    yaml.safe_dump(nav,out);out.close()
    atexit.register(lambda:Path(out.name).unlink(missing_ok=True))
    params=ParameterFile(out.name)
    nodes=[];managed=['controller_server','planner_server','bt_navigator']
    for package,executable in [('nav2_controller','controller_server'),('nav2_planner','planner_server'),('nav2_bt_navigator','bt_navigator')]:
        nodes.append(Node(package=package,executable=executable,name=executable,parameters=[params],
                          remappings=[('cmd_vel','/navigation/cmd_vel_raw')],output='screen'))
    mode=get('mode')
    if mode=='mapping':
        nodes.append(Node(package='slam_toolbox',executable='async_slam_toolbox_node',name='slam_toolbox',
                          parameters=[str(root/'config/slam.yaml')],output='screen'))
    elif mode=='localization':
        mapfile=Path(get('map')).expanduser()
        if not mapfile.is_file():raise ValueError('localization requires map:=/absolute/map.yaml')
        nodes.extend([Node(package='nav2_map_server',executable='map_server',name='map_server',
                           parameters=[params,{'yaml_filename':str(mapfile.resolve())}],output='screen'),
                      Node(package='nav2_amcl',executable='amcl',name='amcl',parameters=[params],output='screen')])
        managed=['map_server','amcl']+managed
    nodes.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='navigation_lifecycle',
                      parameters=[{'autostart':True,'node_names':managed,'bond_timeout':4.}],output='screen'))
    if get('publish_odom_tf')=='true':
        nodes.append(Node(package='navigation_bringup',executable='odom_tf',output='screen'))
    sensors=yaml.safe_load(Path(get('sensors_config')).read_text())
    for name,s in sensors.items():
        if not s['confirmed']:continue
        values=s['xyz']+s['quaternion_xyzw']
        if len(values)!=7 or not all(math.isfinite(v) for v in values):raise ValueError('Invalid sensor transform')
        if abs(sum(v*v for v in values[3:])-1)>.01:raise ValueError('Non-unit sensor quaternion')
        nodes.append(Node(package='tf2_ros',executable='static_transform_publisher',name='mount_'+name,
                          arguments=[str(v) for v in values]+[s['parent'],s['child']],output='screen'))
    nodes.extend([Node(package='navigation_bringup',executable='follow_goal',parameters=[get('follow_config')],output='screen'),
                  Node(package='navigation_bringup',executable='velocity_adapter',output='screen'),
                  Node(package='motion_guard',executable='guard',parameters=[get('safety_config'),{
                      'base_frame':'base_footprint','expected_source':follow['expected_source'],
                      'target_frame':follow['target_frame'],'motion_enabled':get('motion_enabled')=='true'}],output='screen')])
    if get('with_radar')=='true':
        nodes.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(
            get_package_share_directory('perception_bringup'))/'launch/radar.launch.py'))))
    return nodes


def generate_launch_description():
    root=Path(get_package_share_directory('navigation_bringup'))
    args=[DeclareLaunchArgument('mode',default_value='mapping',choices=['mapping','localization','external']),
          DeclareLaunchArgument('map',default_value=''),
          DeclareLaunchArgument('motion_enabled',default_value='false',choices=['true','false']),
          DeclareLaunchArgument('with_radar',default_value='false',choices=['true','false']),
          DeclareLaunchArgument('publish_odom_tf',default_value='true',choices=['true','false']),
          DeclareLaunchArgument('safety_config',default_value=str(Path(get_package_share_directory('motion_guard'))/'config/safety.yaml'))]
    for key,file in [('follow_config','follow.yaml'),('sensors_config','sensors.yaml')]:
        args.append(DeclareLaunchArgument(key,default_value=str(root/'config'/file)))
    return LaunchDescription(args+[OpaqueFunction(function=start)])
