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


def test_robot_forces_guard_bench_disables_it(tmp_path):
    robot=run(tmp_path,WITH_CHASSIS='true',WITH_TONGUE_MODE='true')
    assert robot.returncode==1 and '+ WITH_FOLLOWER=true' in robot.stderr and '+ CAPTURE_SAFETY_PROFILE=robot' in robot.stderr
    bench=run(tmp_path)
    assert bench.returncode==1 and '+ CAPTURE_SAFETY_PROFILE=bench' in bench.stderr
    disabled=run(tmp_path,WITH_CAPTURE_TASKS='false',WITH_VOICE='false',WITH_FOLLOWER='true')
    assert disabled.returncode==1 and '+ CAPTURE_SAFETY_PROFILE=' not in disabled.stderr
