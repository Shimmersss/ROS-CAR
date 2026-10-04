"""Real Qwen3/Silero node consumes ROS PCM and discards a canceled utterance."""
from pathlib import Path
import time
import wave
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from std_srvs.srv import Trigger
from roscar_interfaces.msg import AudioFrame
from offline_voice.asr_node import OfflineAsrNode


def main():
    rclpy.init(args=['--ros-args', '-p', 'audio_source:=topic', '-p', 'asr_backend:=qwen3'])
    probe = Node('asr_replay_test')
    asr = OfflineAsrNode()
    ex = SingleThreadedExecutor(); ex.add_node(probe); ex.add_node(asr)
    audio = probe.create_publisher(AudioFrame, '/voice/audio', 40)
    texts, states = [], []
    probe.create_subscription(String, '/voice/asr_text', lambda m: texts.append(m.data), 10)
    probe.create_subscription(String, '/voice/asr_state', lambda m: states.append(m.data), 10)
    start = probe.create_client(Trigger, '/voice/start_listening')
    cancel = probe.create_client(Trigger, '/voice/cancel_listening')
    sequence = 0

    def pump(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            ex.spin_once(timeout_sec=.003)

    def call(client):
        assert client.wait_for_service(timeout_sec=2)
        future = client.call_async(Trigger.Request())
        while not future.done():
            pump(.01)
        assert future.result().success, future.result().message

    def replay(data):
        nonlocal sequence
        for offset in range(0, len(data), 1280):
            frame = AudioFrame(); frame.stamp = probe.get_clock().now().to_msg()
            frame.sequence, frame.sample_rate, frame.channels = sequence, 16000, 1
            frame.pcm_s16le = list(data[offset:offset + 1280].ljust(1280, b'\0'))
            audio.publish(frame); sequence += 1; pump(.04)

    try:
        pump(.5)
        wav = Path(__file__).resolve().parents[1] / 'artifacts/voice-lab/synthetic/stop.wav'
        with wave.open(str(wav)) as stream:
            data = stream.readframes(stream.getnframes())
        call(start); pump(.1)
        replay(data[:12800]); call(cancel); pump(.5)
        assert not texts, texts
        call(start); pump(.1); replay(data); pump(2)
        assert texts and '停止' in texts[-1], (texts, states)
        assert 'SPEECH' in states and 'RECOGNIZING' in states
        print('PASS real Qwen3 CPU2 + Silero + shared ROS audio: canceled PCM yields no text; complete stop WAV yields 停止')
    finally:
        ex.remove_node(asr); asr.destroy_node()
        ex.remove_node(probe); probe.destroy_node()
        ex.shutdown(); rclpy.shutdown()


if __name__ == '__main__':
    main()
