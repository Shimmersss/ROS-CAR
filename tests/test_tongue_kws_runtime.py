"""Real CPU KWS cancels an ordered tongue prompt; simulated RGB/TTS/cloud.

The known-positive WAV starts in a reset stream. This verifies integration,
not continuous-stream keyword recall. No ALSA, car or real cloud is opened.
"""
import json
import os
from pathlib import Path
import tempfile
import threading
import time
import wave
from http.server import ThreadingHTTPServer
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from roscar_interfaces.msg import AudioFrame
from motion_guard.node import MotionGuard
from offline_voice.emergency_node import EmergencyKwsNode
from voice_command_router.router_node import VoiceCommandRouter
from execution_mode_manager.node import ModeManager
from tongue_mode.node import TongueMode
from test_tongue_mode_runtime import Handler, FakeTts, wait


class PromptTts(FakeTts):
    playing = False

    def _publish_speaking(self, value):
        self.playing = value
        super()._publish_speaking(value)

    def _speak(self, text):
        self.spoken.append(text)
        self._publish_speaking(True)
        end = time.monotonic() + (2. if '正对相机' in text else .06)
        while time.monotonic() < end and not self._speech_cancelled():
            time.sleep(.005)


class FixtureKws(EmergencyKwsNode):
    reset_at_sequence = None

    def on_audio(self, message):
        if message.sequence == self.reset_at_sequence:
            self.engine.reset()
        super().on_audio(message)


class Inputs(Node):
    def __init__(self, kws):
        super().__init__('tongue_kws_inputs')
        self.kws, self.sequence, self.pcm, self.offset = kws, 0, None, 0
        self.lock = threading.Lock()
        self.bridge = CvBridge()
        self.audio = self.create_publisher(AudioFrame, '/voice/audio', 20)
        self.camera = self.create_publisher(Image, '/camera/color/image_raw', qos_profile_sensor_data)
        self.asr = self.create_publisher(String, '/voice/asr_text', 10)
        self.wake = self.create_publisher(String, '/voice_words', 10)
        self.create_timer(.04, self.frame)

    def frame(self):
        with self.lock:
            stamp = self.get_clock().now().to_msg()
            chunk = bytes(1280)
            if self.pcm is not None and self.offset < len(self.pcm):
                chunk = self.pcm[self.offset:self.offset+1280].ljust(1280, b'\0')
                self.offset += 1280
            message = AudioFrame(stamp=stamp, sequence=self.sequence, sample_rate=16000,
                                 channels=1, pcm_s16le=list(chunk))
            self.sequence += 1
            self.audio.publish(message)
            image = self.bridge.cv2_to_imgmsg(np.full((32, 48, 3), 120, np.uint8), encoding='rgb8')
            image.header.stamp = stamp
            self.camera.publish(image)

    def play(self, data):
        with self.lock:
            self.kws.reset_at_sequence = self.sequence
            self.pcm, self.offset = data, 0


def main():
    cloud = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    cloud.daemon_threads = True
    cloud.status, cloud.delay, cloud.uploads, cloud.wrong_version = 'ok', 0., [], False
    threading.Thread(target=cloud.serve_forever, daemon=True).start()
    catalog = Path(__file__).resolve().parents[1]/'ros2_ws/src/execution_mode_manager/config/modes.yaml'
    with tempfile.TemporaryDirectory() as temp:
        rclpy.init(args=['--ros-args', '-p', 'catalog:='+str(catalog),
                        '-p', 'execution_modes_config:='+str(catalog), '-p', 'enable_speech_action:=true',
                        '-p', 'safety_profile:=bench', '-p', 'storage_dir:='+temp+'/results',
                        '-p', 'estop_latch_path:='+temp+'/estop.json',
                        '-p', 'server_url:=http://127.0.0.1:'+str(cloud.server_port)])
        guard, tts, kws = MotionGuard(), PromptTts(), FixtureKws()
        router, manager, tongue = VoiceCommandRouter(), ModeManager(), TongueMode()
        inputs = Inputs(kws)
        detected_while_playing = []
        accept = kws.engine.accept
        def observed(samples):
            keyword = accept(samples)
            if keyword:
                detected_while_playing.append(tts.playing)
            return keyword
        kws.engine.accept = observed
        nodes = [guard, tts, kws, router, manager, tongue, inputs]
        executor = MultiThreadedExecutor(num_threads=10)
        for node in nodes:
            executor.add_node(node)
        thread = threading.Thread(target=executor.spin, daemon=True); thread.start()
        try:
            wait(lambda:guard.emergency_healthy() and tongue.speech.server_is_ready()
                 and manager.mode_clients['tongue'].server_is_ready(), 'graph/real KWS not ready')
            inputs.wake.publish(String(data='小车唤醒')); time.sleep(.2)
            inputs.asr.publish(String(data='舌苔检测'))
            wait(lambda:tongue.ctx and tongue.ctx.phase=='PROMPT' and tts.playing, 'ordered prompt did not start')
            wav = Path(os.environ['VOICE_TEST_STOP_WAV'])
            with wave.open(str(wav)) as stream:
                assert (stream.getframerate(), stream.getnchannels(), stream.getsampwidth()) == (16000, 1, 2)
                inputs.play(stream.readframes(stream.getnframes()))
            wait(lambda:guard.authority.state=='ESTOP', 'real KWS failed to latch stop')
            wait(lambda:manager.active is None and tongue.ctx is None and not guard.task_hold.token,
                 'emergency did not finish mode cleanup')
            assert detected_while_playing and detected_while_playing[0]
            assert not cloud.uploads and guard.command_mode=='IDLE'
            wait(lambda:tts._action_speech.reserved==0 and not tts.playing, 'ordered TTS not cancelled')
            print('PASS real CPU KWS known-positive fixture during SpeakText prompt -> ESTOP -> cancelled speech/mode -> no capture/upload -> no movement restored')
        finally:
            manager.pending = None
            if manager.active:
                manager.exit_active()
            tongue.stop(); tts._stop.set(); tts._action_speech.stop()
            tts._worker.join(timeout=1.)
            for node in nodes:
                for timer in node.timers:
                    timer.cancel()
            time.sleep(.2)  # Drain action cancel/result replies before teardown.
            executor.shutdown(timeout_sec=3.)
            thread.join(timeout=3.)
            for node in nodes:
                executor.remove_node(node)
                node.destroy_node()
            rclpy.shutdown()
            cloud.shutdown(); cloud.server_close()


if __name__ == '__main__':
    main()
