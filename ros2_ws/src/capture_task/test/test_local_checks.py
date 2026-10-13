"""Preflight remains fail-closed with a standard PATH and no ROS dependency."""
import os
from pathlib import Path
import shlex
import subprocess
import pytest
ROOT=Path(__file__).resolve().parents[4]


def check(script):
    return subprocess.run(['bash','-c','source '+shlex.quote(str(ROOT/'scripts/capture_local_checks.sh'))+'\n'+script],
        env={**os.environ,'PATH':'/usr/bin:/bin'},text=True,capture_output=True,timeout=3)


@pytest.mark.parametrize('output,code,message', [
    ('/voice/speak',0,'已存在 SpeakText'),('/other\n/voice/speak',0,'已存在 SpeakText'),
    ('',1,'无法查询 ROS action')])
def test_owner_or_failed_query_rejects(output,code,message):
    script='ros2() { printf "%s\\n" '+shlex.quote(output)+'; return '+str(code)+'; }; capture_check_speech_owner'
    result=check(script)
    assert result.returncode==1 and message in result.stderr


def test_no_owner_succeeds():
    assert check("ros2() { printf '/other\\n'; }; capture_check_speech_owner").returncode==0


def test_header_bug_rejects_without_overlay(tmp_path):
    header=tmp_path/'utils.hpp';header.write_text('epoch_time.tv_usec / 1000.0')
    call='capture_check_usb_cam_timestamp '+shlex.quote(str(header))+' '+shlex.quote(str(tmp_path/'overlay'))
    result=check(call)
    assert result.returncode==1 and '时间戳单位问题' in result.stderr
    header.write_text('epoch_time.tv_usec;')
    assert check(call).returncode==0
    header.unlink()
    assert check(call).returncode==1
