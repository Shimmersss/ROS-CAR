"""Exercise early launcher gates without starting ROS or accessing hardware."""
import os
from pathlib import Path
import subprocess
import pytest

ROOT=Path(__file__).resolve().parents[4]


def run(tmp_path,**changes):
    env=dict(os.environ)
    env.update(PERCEPTION_ROUTE='b',WITH_VOICE='true',WITH_CHASSIS='false',WITH_RADAR='false',
               WITH_FOLLOWER='false',WITH_TONGUE_MODE='false',WITH_CAPTURE_TASKS='true',
               MOTION_ENABLED='false',VOICE_TTS_ENABLED='true',AUTO_LOCK_SINGLE='true',DEPTH_REGISTERED='false',
               MODEL_PATH=str(tmp_path/'absent.engine'),YOLO_PYTHON='/usr/bin/python3')
    env.update(changes)
    return subprocess.run(['bash','-x',str(ROOT/'scripts/start_project.sh')],env=env,capture_output=True,text=True,timeout=3)


@pytest.mark.parametrize('changes',[
    {'WITH_VOICE':'false'},{'VOICE_TTS_ENABLED':'false'},
    {'WITH_FOLLOWER':'true'},{'MOTION_ENABLED':'true'},{'WITH_CAPTURE_TASKS':'yes'}])
def test_reject_unsafe_startup(tmp_path,changes):
    assert run(tmp_path,**changes).returncode==2


def test_robot_requires_explicit_guard_bench_disables_it(tmp_path):
    missing=run(tmp_path,WITH_CHASSIS='true')
    assert missing.returncode==2 and '请显式设置 WITH_FOLLOWER=true' in missing.stderr
    robot=run(tmp_path,WITH_CHASSIS='true',WITH_TONGUE_MODE='true',WITH_FOLLOWER='true')
    assert robot.returncode==1 and '+ CAPTURE_SAFETY_PROFILE=robot' in robot.stderr
    bench=run(tmp_path)
    assert bench.returncode==1 and '+ CAPTURE_SAFETY_PROFILE=bench' in bench.stderr
    disabled=run(tmp_path,WITH_CAPTURE_TASKS='false',WITH_VOICE='false',WITH_FOLLOWER='true',FOLLOW_EMERGENCY_KWS_REQUIRED='false')
    assert disabled.returncode==1 and '+ CAPTURE_SAFETY_PROFILE=' not in disabled.stderr


@pytest.mark.parametrize('changes',[
    {'CAPTURE_ROS_API':'yes'}, {'CAPTURE_ROS_API':'true','WITH_CAPTURE_TASKS':'false'}])
def test_ros_api_requires_capture_and_boolean(tmp_path,changes):
    assert run(tmp_path,**changes).returncode==2


def test_ros_api_default_closed_and_explicit_gate(tmp_path):
    default=run(tmp_path,CAPTURE_ROS_API='false')
    assert default.returncode==1 and '+ CAPTURE_ROS_API=false' in default.stderr
    enabled=run(tmp_path,CAPTURE_ROS_API='true')
    assert enabled.returncode==1 and '+ CAPTURE_ROS_API=true' in enabled.stderr


@pytest.mark.parametrize('changes',[
    {'CAPTURE_PIPELINE_MODE':'infer','CAPTURE_INFERENCE_URL':''},
    {'CAPTURE_PIPELINE_MODE':'invalid'},
    {'CAPTURE_RETURN_ORIGINAL_IMAGE':'yes'},
])
def test_inference_requires_explicit_valid_configuration(tmp_path,changes):
    result=run(tmp_path,**changes)
    assert result.returncode==2
    assert 'start_module' not in result.stderr


def test_inference_url_passes_early_gate(tmp_path):
    result=run(tmp_path,CAPTURE_PIPELINE_MODE='infer',CAPTURE_INFERENCE_URL='http://127.0.0.1:8200')
    assert result.returncode==1 and '缺少 b 模型' in result.stderr


@pytest.mark.parametrize('follow_options', [
    {},
    {'FOLLOW_AUTO_ARM': 'true', 'FOLLOW_TARGET_AUTO_RECOVER': 'true',
     'FOLLOW_EMERGENCY_KWS_REQUIRED': 'false'},
])
def test_capture_robot_preserves_explicit_authorization(tmp_path, follow_options):
    result = run(tmp_path, WITH_CHASSIS='true', WITH_FOLLOWER='true',
                 MOTION_ENABLED='true', **follow_options)
    assert result.returncode == 1  # Stop at the deliberately missing model.
    assert '+ CAPTURE_SAFETY_PROFILE=robot' in result.stderr
    assert '+ FOLLOW_AUTO_ARM=false' in result.stderr
    assert '+ FOLLOW_TARGET_AUTO_RECOVER=false' in result.stderr
    assert '+ FOLLOW_EMERGENCY_KWS_REQUIRED=true' in result.stderr


def test_capture_disabled_preserves_upstream_follow_options(tmp_path):
    result = run(tmp_path, WITH_CAPTURE_TASKS='false', WITH_FOLLOWER='true',
                 MOTION_ENABLED='true', FOLLOW_AUTO_ARM='true',
                 FOLLOW_TARGET_AUTO_RECOVER='true', FOLLOW_EMERGENCY_KWS_REQUIRED='true')
    assert result.returncode == 1
    assert '+ FOLLOW_AUTO_ARM=true' in result.stderr
    assert '+ FOLLOW_TARGET_AUTO_RECOVER=true' in result.stderr
    assert '+ FOLLOW_AUTO_ARM=false' not in result.stderr
    assert '+ FOLLOW_TARGET_AUTO_RECOVER=false' not in result.stderr
