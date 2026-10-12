"""Real Humble nodes/services + CPU KWS, synthetic audio and vendor WAV.

No ALSA, serial driver or car. Geometry/calibration flags below are test-only.
"""
import json
import os
from pathlib import Path
import tempfile
import time
import wave
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from roscar_interfaces.msg import AudioFrame
from roscar_interfaces.srv import SetControlMode, AuthorizeMotion
from motion_guard.node import MotionGuard
from voice_command_router.router_node import VoiceCommandRouter
from voice_command_router.motion_adapter_node import MotionAdapter
from offline_voice.emergency_node import EmergencyKwsNode
from xfyun_speech.tts_node import XfyunTtsNode
from astra_body_adapter.person_follower_node import PersonFollowerNode
from person_interfaces.msg import TargetState


class FakePlayback(XfyunTtsNode):
    def __init__(self):
        self.played = []
        self.playing = False
        super().__init__()

    def _publish_speaking(self, value):
        self.playing = bool(value)
        return super()._publish_speaking(value)

    def _speak(self, text):
        self.played.append(text)
        self._publish_speaking(True)
        end = time.monotonic() + 2
        while time.monotonic() < end and not self._speech_cancelled():
            time.sleep(.01)


def main():
    rclpy.init()
    ex = SingleThreadedExecutor()
    probe = Node('voice_authority_test')
    params = dict(motion_enabled=True, geometry_confirmed=True,
                  mount_calibrated=True, stopping_model_confirmed=True,
                  estop_latch_path=str(Path(tempfile.mkdtemp()) / 'estop.json'))
    guard = MotionGuard(parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])
    adapter = MotionAdapter(parameter_overrides=[Parameter('turn_calibrated', value=True), Parameter('micro_calibrated', value=True)])
    router = VoiceCommandRouter()
    kws = EmergencyKwsNode()
    playback = FakePlayback()
    follower = PersonFollowerNode(parameter_overrides=[Parameter('enabled', value=True)])
    nodes = (probe, guard, adapter, router, kws, playback, follower)
    for node in nodes:
        ex.add_node(node)
    targets = probe.create_publisher(TargetState, '/perception/target_state', 1)
    audio = probe.create_publisher(AudioFrame, '/voice/audio', 20)
    asr = probe.create_publisher(String, '/voice/asr_text', 10)
    wake = probe.create_publisher(String, '/voice_words', 10)
    tts = probe.create_publisher(String, '/voice/tts_text', 10)
    states, velocities, events = [], [], []
    probe.create_subscription(String, '/control/state', lambda m: states.append(json.loads(m.data)), 10)
    probe.create_subscription(Twist, '/cmd_vel', lambda m: velocities.append((m.linear.x, m.angular.z)), 10)
    probe.create_subscription(String, '/voice/emergency_event', lambda m: events.append(dict(json.loads(m.data), tts_was_playing=playback.playing)), 10)
    counter = 0
    next_audio = time.monotonic()

    def publish_frame(pcm):
        nonlocal counter
        frame = AudioFrame()
        frame.stamp = probe.get_clock().now().to_msg()
        frame.sequence, frame.sample_rate, frame.channels = counter, 16000, 1
        frame.pcm_s16le = list(pcm)
        target = TargetState(); target.header.stamp = frame.stamp
        target.header.frame_id = 'astra_depth_optical_frame'; target.source = 'astra'
        target.status = TargetState.TRACKING; target.position_valid = True
        target.position.z = 3.; target.horizontal_distance_m = 3.; target.measurement_age_s = float('nan')
        targets.publish(target)
        audio.publish(frame)
        counter += 1

    def pump(seconds, feed=True):
        nonlocal next_audio
        end = time.monotonic() + seconds
        if next_audio < time.monotonic() - .1:
            next_audio = time.monotonic()
        while time.monotonic() < end:
            if feed and time.monotonic() >= next_audio:
                publish_frame(bytes(1280))
                next_audio += .04
            ex.spin_once(timeout_sec=.003)

    def call(topic, cls=Trigger, request=None):
        client = probe.create_client(cls, topic)
        assert client.wait_for_service(timeout_sec=2), topic
        future = client.call_async(request or cls.Request())
        end = time.monotonic() + 3
        while not future.done() and time.monotonic() < end:
            pump(.02)
        assert future.done(), topic
        result = future.result()
        probe.destroy_client(client)
        return result

    def stopped():
        assert velocities and all(v == (0., 0.) for v in velocities[-3:]), velocities[-3:]

    def authorize(mode='EXTERNAL'):
        request = SetControlMode.Request(); request.mode = mode
        result = call('/control/set_mode', SetControlMode, request)
        assert result.success, result.message
        pump(.3)
        ticket = AuthorizeMotion.Request()
        ticket.mode, ticket.authority_epoch = mode, result.authority_epoch
        result = call('/control/authorize', AuthorizeMotion, ticket)
        assert result.success, (result.message, states[-1])
        pump(.1)
        assert states[-1]['authority_state'] == ('ARMED_MANUAL' if mode == 'EXTERNAL' else 'ARMED_FOLLOW')
        return ticket

    try:
        pump(1.5)
        assert kws.engine and states[-1]['authority_state'] == 'DISARMED'
        stopped()
        assert not call('/control/arm').success
        asr.publish(String(data='前进')); pump(.2); stopped()
        wake.publish(String(data='小车唤醒')); pump(.2)
        wake.publish(String(data='小车唤醒')); pump(.2)
        assert router.session.active(time.monotonic()) and states[-1]['authority_state'] == 'DISARMED'
        ticket = authorize()
        asr.publish(String(data='前进速度0.2两秒')); pump(.2)
        assert any(v[0] == .2 for v in velocities[-5:]), velocities[-5:]
        asr.publish(String(data='左转速度0.2两秒')); pump(.2)
        assert any(v == (.08, .2) for v in velocities[-5:]), velocities[-5:]
        asr.publish(String(data='左转一下')); pump(.3)
        assert any(v[0] == .05 and v[1] == .1 for v in velocities[-10:]), velocities[-10:]
        pump(.6); stopped()
        tts.publish(String(data='请继续说明')); pump(.15)
        assert playback.playing
        # Isolate the known-positive fixture from prior synthetic silence.
        # This tests event delivery during TTS, not KWS recall at arbitrary
        # stream offsets; offset-dependent misses are recorded separately.
        kws.engine.reset()
        # TTS active flag is irrelevant to the separate KWS subscription.
        wav = Path(os.environ.get('VOICE_TEST_STOP_WAV', str(Path(__file__).resolve().parents[1] / 'artifacts/voice-lab/synthetic/stop.wav')))
        with wave.open(str(wav)) as stream:
            assert (stream.getframerate(), stream.getnchannels(), stream.getsampwidth()) == (16000, 1, 2)
            data = stream.readframes(stream.getnframes())
        for offset in range(0, len(data), 1280):
            # Each WAV chunk must be published exactly once, independent of
            # the silence timer phase and executor scheduling jitter.
            publish_frame(data[offset:offset + 1280].ljust(1280, b'\0'))
            next_audio = time.monotonic() + .04
            pump(.04, feed=False)
        pump(.6)
        assert events and events[0]['tts_was_playing'] and states[-1]['authority_state'] == 'ESTOP', (events, states[-1])
        stopped()
        assert call('/control/stop').success
        assert not call('/control/authorize', AuthorizeMotion, ticket).success
        assert not call('/control/set_mode', SetControlMode, SetControlMode.Request(mode='EXTERNAL')).success
        assert call('/control/reset_estop').success
        pump(.2)
        assert states[-1]['authority_state'] == 'DISARMED' and states[-1]['command_mode'] == 'IDLE'
        stopped()
        ticket = authorize()
        pump(.8, feed=False)
        assert states[-1]['authority_state'] == 'FAULT'
        stopped()
        pump(.3)
        assert states[-1]['authority_state'] == 'FAULT'
        assert call('/control/stop').success
        assert call('/control/reset_fault').success
        pump(.2); stopped()
        assert states[-1]['authority_state'] == 'DISARMED'
        authorize('FOLLOW'); pump(.3)
        assert any(v[0] > 0 for v in velocities[-5:])
        assert call('/control/stop').success
        pump(.3); stopped()
        assert states[-1]['authority_state'] == 'DISARMED'
        ticket = authorize()
        router._arm_inflight = True  # Inject a delayed authorization completion.
        router._close_session()
        pump(.3); stopped()
        assert states[-1]['authority_state'] == 'DISARMED'
        assert not call('/control/authorize', AuthorizeMotion, ticket).success
        # Canceled TTS jobs are drained; a new generation can still speak.
        assert call('/voice/cancel_speech').success
        assert playback._active_cancel.is_set()
        print('PASS real ROS authority/adapter/router/KWS: boot disarmed, wake refresh, zero handshake, numeric limits, micro curve, isolated positive KWS fixture during TTS, latch/reset, stale ticket, audio-loss FAULT, explicit recovery, speech cancellation')
    finally:
        playback._stop.set()
        for node in reversed(nodes):
            ex.remove_node(node)
            node.destroy_node()
        ex.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
