#!/usr/bin/env python3
"""Real Humble nodes/interfaces; fake RGB, odometry, audio playback and cloud model.

No physical camera, microphone, vehicle or learned model is used by this test.
"""
import json
import os
from pathlib import Path
import socket
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from roscar_interfaces.srv import SetControlMode, AuthorizeMotion
from execution_mode_interfaces.action import RunMode, SpeakText
from execution_mode_interfaces.msg import ModeCommand
from execution_mode_manager.node import ModeManager
from tongue_mode.node import TongueMode
from tongue_mode.results import LABELS
from voice_command_router.router_node import VoiceCommandRouter
from motion_guard.node import MotionGuard
from xfyun_speech.tts_node import XfyunTtsNode

PIPELINE = 'cloud_r50_384_cvAC10_passthrough_debug'
DIGEST = 'c5b9b1e31f73515b44044569b66e64d49861880441273a1fb86cd84774d6a494'


def wait(predicate, detail, timeout=5.):
    end = time.monotonic()+timeout
    while time.monotonic() < end:
        if predicate(): return
        time.sleep(.025)
    raise AssertionError(detail)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args): pass
    def send(self, body):
        raw = json.dumps(body).encode()
        try:
            self.send_response(200); self.send_header('Content-Length', str(len(raw))); self.end_headers(); self.wfile.write(raw)
        except (BrokenPipeError, ConnectionResetError): pass
    def do_GET(self):
        if self.path == '/health/ready': self.send({'status':'ready'})
        else: self.send(dict(pipeline_id=PIPELINE, pipeline_digest='wrong' if self.server.wrong_version else DIGEST,
                             labels=list(LABELS), device='cuda'))
    def do_POST(self):
        raw = self.rfile.read(int(self.headers['Content-Length']))
        assert raw[:2] == b'\xff\xd8' and self.headers['Content-Type'] == 'image/jpeg'
        self.server.uploads.append(dict(request_id=self.headers['X-Request-Id'], capture_id=self.headers['X-Capture-Id']))
        status, delay = self.server.status, self.server.delay
        time.sleep(delay)
        labels = {k:dict(score=.8 if k=='FurYellow' else .1, threshold=.5,
                        decision='positive' if k=='FurYellow' else 'negative') for k in LABELS} if status=='ok' else {}
        self.send(dict(status=status, labels=labels, classification_performed=status=='ok',
                       request_id=self.headers['X-Request-Id'], capture_id=self.headers['X-Capture-Id'],
                       session_id=self.headers['X-Session-Id'], pipeline_id=PIPELINE, pipeline_digest=DIGEST))


class FakeTts(XfyunTtsNode):
    def __init__(self):
        self.spoken = []
        self.fail_next = False
        super().__init__()
    def _speak(self, text):
        self.spoken.append(text)
        self._publish_speaking(True); self._publish_state('SPEAKING')
        end = time.monotonic()+.06
        while time.monotonic() < end and not self._speech_cancelled(): time.sleep(.005)
        if self.fail_next:
            self.fail_next = False
            raise RuntimeError('simulated playback failure')


class IO(Node):
    def __init__(self):
        super().__init__('tongue_test_io')
        self.camera_enabled = True
        self.emergency_enabled = True
        self.states, self.results, self.mode_states = [], [], []
        self.bridge = CvBridge()
        self.images = self.create_publisher(Image, '/camera/color/image_raw', qos_profile_sensor_data)
        self.odometry = self.create_publisher(Odometry, '/odom', 2)
        self.asr = self.create_publisher(String, '/voice/asr_text', 10)
        self.wake = self.create_publisher(String, '/voice_words', 10)
        self.emergency_health = self.create_publisher(Bool, '/voice/emergency_healthy', 10)
        self.emergency = self.create_publisher(Bool, '/control/emergency_stop', 10)
        self.create_subscription(String, '/tongue/result', lambda m:self.results.append(json.loads(m.data)), 10)
        self.create_subscription(String, '/execution_mode/state', lambda m:self.mode_states.append(json.loads(m.data)), 10)
        self.create_subscription(String, '/tongue/state', lambda m:self.states.append(json.loads(m.data)), 10)
        self.create_timer(.04, self.publish_inputs)
        self.set_mode = self.create_client(SetControlMode, '/control/set_mode')
        self.arm = self.create_client(Trigger, '/control/arm')
        self.authorize = self.create_client(AuthorizeMotion, '/control/authorize')
        self.reset_estop = self.create_client(Trigger, '/control/reset_estop')
    def publish_inputs(self):
        self.emergency_health.publish(Bool(data=self.emergency_enabled))
        if self.camera_enabled:
            data = np.full((32, 48, 3), (220, 90, 100), np.uint8)
            msg = self.bridge.cv2_to_imgmsg(data, encoding='rgb8')
            msg.header.stamp = self.get_clock().now().to_msg()
            self.images.publish(msg)
        odom = Odometry(); odom.header.stamp = self.get_clock().now().to_msg()
        self.odometry.publish(odom)
    def say(self, text, wake=True):
        if wake:
            self.wake.publish(String(data='小车唤醒'))
            time.sleep(.12)
        self.asr.publish(String(data=text))


class OtherMode(Node):
    def __init__(self, guard):
        super().__init__('other_test_mode')
        self.guard = guard
        self.starts = 0
        self.stop_event = threading.Event()
        self.server = ActionServer(self, RunMode, '/modes/other/run', self.run,
                                   goal_callback=lambda _:GoalResponse.ACCEPT,
                                   cancel_callback=lambda _:CancelResponse.ACCEPT,
                                   callback_group=ReentrantCallbackGroup())
    def run(self, goal):
        assert not self.guard.task_hold.token, 'Old mode must release its hold before switching'
        self.starts += 1
        while rclpy.ok() and not goal.is_cancel_requested and not self.stop_event.is_set():
            goal.publish_feedback(RunMode.Feedback(phase='READY'))
            time.sleep(.1)
        if goal.is_cancel_requested: goal.canceled()
        else: goal.abort()
        return RunMode.Result(status='cancelled')


def main():
    cloud = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    cloud.daemon_threads=True; cloud.status='ok'; cloud.delay=0.; cloud.uploads=[]; cloud.wrong_version=False
    threading.Thread(target=cloud.serve_forever, daemon=True).start()
    with tempfile.TemporaryDirectory(prefix='tongue-ros-') as temp:
        import yaml
        catalog = Path(temp)/'modes.yaml'
        share = Path(__file__).resolve().parents[1]/'ros2_ws/src/execution_mode_manager/config/modes.yaml'
        data = yaml.safe_load(share.read_text())
        data['modes'].append(dict(id='other', action='/modes/other/run', enter=['其他模式']))
        catalog.write_text(yaml.safe_dump(data, allow_unicode=True))
        rclpy.init(args=['--ros-args', '-p', 'catalog:='+str(catalog), '-p', 'execution_modes_config:='+str(catalog),
                        '-p', 'server_url:=http://127.0.0.1:'+str(cloud.server_port),
                        '-p', 'storage_dir:='+temp+'/results', '-p', 'enable_speech_action:=true',
                        '-p', 'prepare_sec:=0.02', '-p', 'task_timeout_sec:=5.0',
                        '-p', 'frame_timeout_sec:=0.3', '-p', 'manager_timeout_sec:=1.5',
                        '-p', 'motion_enabled:=true', '-p', 'auto_arm:=false',
                        '-p', 'estop_latch_path:='+temp+'/estop.json',
                        '-p', 'geometry_confirmed:=true', '-p', 'mount_calibrated:=true',
                        '-p', 'stopping_model_confirmed:=true'])
        io, guard, tts, router, manager, tongue = IO(), MotionGuard(), FakeTts(), VoiceCommandRouter(), ModeManager(), TongueMode()
        other = OtherMode(guard)
        nodes = [io, guard, tts, router, manager, tongue, other]
        executor = MultiThreadedExecutor(num_threads=10)
        for node in nodes: executor.add_node(node)
        spinning = threading.Thread(target=executor.spin, daemon=True); spinning.start()
        cases=[]
        def mark(name): cases.append(name); print('PASS:', name, flush=True)
        def ready(count, timeout=8.): wait(lambda:len(io.results)>=count and manager.active and manager.active['phase']=='READY', 'capture did not finish', timeout)
        def exited(): wait(lambda:manager.active is None and not guard.task_hold.token, 'mode did not exit')
        def call(client, request):
            f=client.call_async(request); wait(f.done, 'service timed out'); return f.result()
        try:
            wait(lambda:io.asr.get_subscription_count()>0 and tongue.hold.service_is_ready() and tongue.speech.server_is_ready() and manager.mode_clients['tongue'].server_is_ready(), 'ROS graph discovery failed')
            time.sleep(.5)
            assert guard.authority.state == 'DISARMED'
            io.say('舌苔检测', wake=False); time.sleep(.3)
            assert manager.active is None and not cloud.uploads
            mark('unwoken mode command is rejected; boot remains DISARMED')
            io.say('舌苔检测'); ready(1)
            assert guard.command_mode=='IDLE' and guard.task_hold.token
            assert io.results[0]['response']['labels']['FurYellow']['decision']=='positive'
            assert not any('没有识别到控制指令' in s for s in tts.spoken)
            assert manager.active['mode_id']=='tongue'
            mark('voice entry -> hold -> prompt completion -> fresh JPEG -> validated response -> speech -> persistent READY')

            assert not call(io.set_mode, SetControlMode.Request(mode='FOLLOW')).success
            assert not call(io.arm, Trigger.Request()).success
            io.say('前进'); time.sleep(.2)
            assert guard.command_mode=='IDLE'
            assert not call(io.authorize, AuthorizeMotion.Request(mode='IDLE', authority_epoch=guard.authority.epoch)).success
            mark('motion and epoch authorization cannot bypass active mode hold; repeated wake does not exit mode')

            before=len(cloud.uploads); io.say('播报上次结果')
            wait(lambda:sum('本次识别到黄苔' in s for s in tts.spoken)>=2, 'repeat failed')
            assert len(cloud.uploads)==before
            mark('repeat reads session cache without another upload')

            io.say('重新拍摄'); ready(2)
            assert io.results[0]['ids']['capture_id']!=io.results[1]['ids']['capture_id']
            mark('retake uses a new frame and new identity')

            cloud.status='retake'; io.say('重新拍摄'); ready(3)
            assert not io.results[-1]['response']['labels']
            io.say('播报上次结果'); wait(lambda:any('当前没有可重播' in s for s in tts.spoken), 'retake reused old success')
            mark('retake is not eight fake negatives and does not replay prior success')

            cloud.status='ok'; io.camera_enabled=False; before=len(cloud.uploads)
            io.say('重新拍摄'); wait(lambda:any(s['phase']=='CAPTURE_FAILED' for s in io.states), 'stale frame accepted')
            wait(lambda:manager.active and manager.active['phase']=='READY', 'camera failure did not recover')
            assert len(cloud.uploads)==before and tongue.ctx.last_result is None
            io.camera_enabled=True
            mark('stale cached image is rejected before upload')

            cloud.wrong_version=True; before=len(cloud.uploads)
            states_at=len(io.states); io.say('重新拍摄')
            wait(lambda:any(s['phase']=='CAPTURE_FAILED' for s in io.states[states_at:]), 'wrong model version accepted')
            wait(lambda:manager.active and manager.active['phase']=='READY', 'version failure not recovered')
            assert len(cloud.uploads)==before
            cloud.wrong_version=False
            mark('wrong cloud pipeline never reaches capture/classification speech')

            tts.fail_next=True; before=len(cloud.uploads); states_at=len(io.states)
            io.say('重新拍摄'); wait(lambda:any(s['phase']=='CAPTURE_FAILED' for s in io.states[states_at:]), 'prompt playback failure ignored')
            wait(lambda:manager.active and manager.active['phase']=='READY', 'speech failure not recovered')
            assert len(cloud.uploads)==before
            mark('failed prompt playback blocks capture')

            cloud.delay=.8; before=len(cloud.uploads); results_at=len(io.results)
            io.say('重新拍摄'); wait(lambda:len(cloud.uploads)>before, 'first request of busy retake not sent')
            cloud.delay=0.; io.say('重新拍摄'); ready(results_at+1); time.sleep(.9)
            assert len(io.results)==results_at+1
            assert io.results[-1]['ids']['request_id']==cloud.uploads[-1]['request_id']
            mark('retake during HTTP wait cancels old transport and saves only the new result')

            cloud.delay=.8; before=len(cloud.uploads); results_at=len(io.results)
            io.say('重新拍摄'); wait(lambda:len(cloud.uploads)>before, 'delayed request not sent')
            io.say('取消舌苔检测'); exited(); time.sleep(1.)
            assert len(io.results)==results_at and guard.command_mode=='IDLE'
            cloud.delay=0.
            mark('cancel during network wait drops late result and leaves vehicle IDLE')

            io.say('舌苔检测'); ready(results_at+1)
            io.say('其他模式'); wait(lambda:other.starts==1 and manager.active and manager.active['mode_id']=='other', 'serial mode switch failed')
            assert tongue.ctx is None and not guard.task_hold.token
            io.say('退出当前模式'); exited()
            mark('a second RunMode module is entered only after tongue cleanup')

            io.say('舌苔检测'); ready(results_at+2)
            manager.timer.cancel(); exited()
            assert guard.command_mode=='IDLE'
            manager.timer.reset()
            mark('manager heartbeat loss exits mode and releases the hold without movement')

            # A cloud task outlives listening; READY opens one new 8s window.
            tongue.cfg['task_timeout_sec'] = 15.
            cloud.delay = 8.3
            results_at = len(io.results)
            io.say('舌苔检测'); ready(results_at+1, timeout=15.)
            wait(router._session_active, 'READY failed to reopen bounded listening')
            repeated_at = sum('本次识别到黄苔' in s for s in tts.spoken)
            io.say('播报上次结果', wake=False)
            wait(lambda:sum('本次识别到黄苔' in s for s in tts.spoken)>repeated_at, 'post-cloud follow-up failed')
            cloud.delay = 0.
            mark('long cloud job resumes bounded follow-up after result, without infinite listening')

            cloud.delay = .8; results_at = len(io.results); before=len(cloud.uploads)
            io.say('重新拍摄'); wait(lambda:len(cloud.uploads)>before, 'ESTOP case upload not sent')
            io.emergency.publish(Bool(data=True)); exited(); time.sleep(.9)
            assert len(io.results)==results_at and guard.authority.state=='ESTOP'
            assert call(io.reset_estop, Trigger.Request()).success
            cloud.delay = 0.
            mark('independent emergency event cancels cloud result and preserves ESTOP through lease release')

            io.say('舌苔检测'); ready(results_at+1)
            io.emergency_enabled=False; exited()
            assert guard.command_mode=='IDLE' and guard.authority.state=='DISARMED'
            io.emergency_enabled=True; time.sleep(.15)
            mark('emergency-health loss exits stationary mode without restoring movement')

            io.say('舌苔检测'); ready(results_at+2)
            io.say('退出语音'); exited()
            assert not router._session_active() and guard.authority.state=='DISARMED'
            mark('explicit voice-session exit cleans up tongue mode; ordinary timeout only pauses listening')

            assert len({u['request_id'] for u in cloud.uploads})==len(cloud.uploads)
            print(json.dumps(dict(passed=True, cases=cases, uploads=len(cloud.uploads), result_records=len(io.results),
                                 scope='Real ROS Humble nodes, mock images/audio/odom/cloud; no hardware or model inference'), ensure_ascii=False, indent=2), flush=True)
        finally:
            manager.pending = None
            if manager.active: manager.exit_active()
            tongue.stop(); other.stop_event.set(); tts._stop.set()
            if tts._action_speech: tts._action_speech.stop()
            tts._worker.join(timeout=1.)
            time.sleep(.1)
            executor.shutdown(timeout_sec=3.)
            for node in nodes: node.destroy_node()
            rclpy.shutdown()
            spinning.join(timeout=3.)
            cloud.shutdown(); cloud.server_close()


if __name__=='__main__': main()
