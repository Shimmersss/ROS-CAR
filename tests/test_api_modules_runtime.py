"""Installed public launch/API smoke test with a PTY gimbal simulator; no physical hardware."""
import json
import os
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from execution_mode_interfaces.msg import ModeCommand
from person_interfaces.srv import ClaimOwner
from roscar_interfaces.srv import SayText, SetControlMode, AuthorizeMotion
from roscar_api.interfaces import discover


def main():
    rejected = subprocess.run(['ros2', 'launch', 'roscar_api', 'api.launch.py',
        'with_voice:=true', 'speech_with_tts:=true'], capture_output=True, text=True, timeout=10)
    assert rejected.returncode != 0 and 'single TTS' in rejected.stdout + rejected.stderr
    rclpy.init()
    node = Node('api_modules_test')
    states = []
    node.create_subscription(String, '/control/state', lambda m: states.append(json.loads(m.data)), 10)
    with tempfile.TemporaryFile(mode='w+') as log:
        proc = subprocess.Popen(['ros2', 'launch', 'roscar_api', 'api.launch.py',
            'with_perception:=true', 'route:=yolo_pose', 'with_speech:=true',
            'with_tongue:=true', 'tongue_safety_profile:=bench',
            'with_gimbal:=true', 'gimbal_mock:=true'], stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True)
        def wait(predicate, timeout=12.):
            end = time.monotonic()+timeout
            while time.monotonic() < end:
                assert proc.poll() is None, 'launch exited'
                rclpy.spin_once(node, timeout_sec=.03)
                if predicate():
                    return
            raise AssertionError('public API unavailable')
        def call(kind, path, request):
            client = node.create_client(kind, path)
            wait(client.service_is_ready)
            future = client.call_async(request)
            wait(future.done)
            return future.result()
        try:
            wait(lambda: '/modes/tongue/run' in discover(node)['actions'] and
                 '/gimbal/save_config' in discover(node)['services'] and states)
            graph = discover(node)
            assert graph['actions']['/modes/tongue/run'] == ['execution_mode_interfaces/action/RunMode']
            # Action discovery also lists clients. No playback owner is launched here.
            assert not {'offline_tts', 'xfyun_tts'} & set(node.get_node_names())
            assert node.get_node_names().count('motion_guard') == 1
            assert all(not s['motion_enabled'] and s['command_mode'] == 'IDLE' for s in states)
            assert 'wheeltec_robot' not in node.get_node_names() and 'voice_audio_capture' not in node.get_node_names()
            assert not call(ClaimOwner, '/perception/claim_owner', ClaimOwner.Request()).success
            assert call(Trigger, '/perception/release_target', Trigger.Request()).success
            assert call(SayText, '/voice/say', SayText.Request(text=' ')).status == 'invalid_text'
            end = time.monotonic()+8.
            while True:
                response = call(SayText, '/voice/say', SayText.Request(text='接口测试'))
                if response.status != 'busy':
                    break
                assert time.monotonic() < end, 'speech bridge stayed busy'
                rclpy.spin_once(node, timeout_sec=.2)
            assert response.status == 'unavailable'
            mode = call(SetControlMode, '/control/set_mode', SetControlMode.Request(mode='EXTERNAL'))
            assert mode.success
            assert not call(AuthorizeMotion, '/control/authorize', AuthorizeMotion.Request(
                mode='EXTERNAL', authority_epoch=mode.authority_epoch)).success
            assert call(Trigger, '/control/stop', Trigger.Request()).success
            # Public services are actually callable; the MCU acknowledgement comes from a PTY fixture.
            assert call(Trigger, '/gimbal/set_pan_zero', Trigger.Request()).success
            listing = subprocess.run(['ros2', 'run', 'roscar_api', 'interfaces', '--json', '--wait', '1'],
                capture_output=True, text=True, timeout=6)
            assert listing.returncode == 0, listing.stderr
            assert '/control/task_hold' in json.loads(listing.stdout)['services']
            requests = []
            node.create_subscription(ModeCommand, '/execution_mode/request', requests.append, 10)
            # Spin the new subscriber during CLI discovery and publication.
            command = subprocess.Popen(['ros2', 'run', 'roscar_api', 'mode', 'EXIT', '--mode', 'tongue'],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            try:
                wait(lambda: command.poll() is not None and bool(requests), timeout=8.)
            finally:
                if command.poll() is None:
                    command.terminate()
                output, error = command.communicate(timeout=3)
            assert command.returncode == 0 and '请求已发送' in output, error
            assert requests[-1].operation == 'EXIT' and requests[-1].mode_id == 'tongue'
            assert requests[-1].request_id and requests[-1].header.stamp.sec > 0
            print('PASS public API modules: C/owner/speech/mode/gimbal graph, native service calls, single guard, no physical I/O')
        except Exception:
            log.seek(0)
            print(log.read())
            raise
        finally:
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
                try:
                    proc.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait(timeout=3)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
