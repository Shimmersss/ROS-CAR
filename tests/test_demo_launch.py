"""Evaluate the actual offline launch conditions without launching audio/hardware."""
import importlib.util
from pathlib import Path
from launch import LaunchContext
from launch_ros.actions import Node

root = Path(__file__).resolve().parents[1]
path = root/'ros2_ws/src/offline_voice/launch/offline_voice.launch.py'
spec = importlib.util.spec_from_file_location('offline_launch', path)
module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
module.get_package_share_directory = lambda _: str(path.parent.parent)
launch = module.generate_launch_description()
adapter, = [action for action in launch.entities if isinstance(action, Node)
            and action.node_executable == 'motion_adapter']
context = LaunchContext()
for action in launch.entities:
    if getattr(action, 'name', None) == 'with_motion_adapter': action.execute(context)
assert adapter.condition.evaluate(context) is True
context.launch_configurations['with_motion_adapter'] = 'false'
assert adapter.condition.evaluate(context) is False
print('PASS: voice adapter enabled by default, disabled explicitly in demo')

# Evaluate follow launch parameters: the final guard overrides grace during fall pause.
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
from launch_ros.utilities import evaluate_parameters
path=root/'ros2_ws/src/motion_guard/launch/follow.launch.py'
spec=importlib.util.spec_from_file_location('follow_launch',path)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
module.get_package_share_directory=lambda _:str(path.parent.parent)
context=LaunchContext()
for action in module.generate_launch_description().entities:
    if isinstance(action,DeclareLaunchArgument):action.execute(context)
nodes=module.start(context)
parameters=evaluate_parameters(context,nodes[0]._Node__parameters)
assert parameters[0]['pause_on_fall'] is False
context.launch_configurations.update(pause_on_fall='true',target_grace_s='0.0')
nodes=module.start(context)
parameters=evaluate_parameters(context,nodes[0]._Node__parameters)
assert parameters[0]['pause_on_fall'] is True and parameters[0]['fall_resume_s']==3.
context.launch_configurations['target_grace_s']='0.6'
nodes=module.start(context)
parameters=evaluate_parameters(context,nodes[1]._Node__parameters)
assert parameters[1]['pause_on_fall'] is True and parameters[1]['target_grace_s']==.6
print('PASS: fall pause is wired to final guard independently of target grace')
