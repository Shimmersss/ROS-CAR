"""Continuous microphone capture with VAD and a separate Qwen3-ASR worker."""

import os
from pathlib import Path
import queue
import threading
import time
import traceback

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from xfyun_speech.audio import ArecordCapture

from .continuous_audio import VadSegmenter


MODEL_ROOT = Path(os.environ.get(
    'ROSCAR_OFFLINE_MODEL_ROOT', '/home/wheeltec/ROSCAR-offline/models'))


class ContinuousAsrNode(Node):
    def __init__(self):
        super().__init__('offline_asr')
        self.declare_parameter('capture_device', 'default')
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('frame_ms', 40)
        self.declare_parameter('asr_backend', 'qwen3')
        self.declare_parameter('qwen3_model_dir', str(
            MODEL_ROOT / 'sherpa-onnx-qwen3-asr-0.6B-int8-2026-03-25'))
        self.declare_parameter('vad_model', str(MODEL_ROOT / 'silero_vad.onnx'))
        self.declare_parameter('vad_threshold', 0.5)
        self.declare_parameter('vad_min_speech_s', 0.12)
        self.declare_parameter('vad_silence_s', 0.7)
        self.declare_parameter('vad_max_speech_s', 8.0)
        self.declare_parameter('tts_cooldown_s', 0.2)
        self.declare_parameter('max_result_age_s', 6.0)
        self.declare_parameter('num_threads', 2)
        if self.get_parameter('asr_backend').value != 'qwen3':
            raise ValueError('连续收音入口只支持 qwen3；其他模型使用旧 ASR 入口')

        self._recognizer = self._load_recognizer()
        self._vad = self._load_vad()
        self._segments = VadSegmenter(self._vad, self._vad.config.silero_vad.window_size)
        self._jobs = queue.Queue(maxsize=2)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._active = False
        self._epoch = 0
        self._speaking = False
        self._mute_until = 0.0
        self._capture = None
        self._state_value = 'IDLE'
        self._text_pub = self.create_publisher(
            String, '/voice/asr_text', 10)
        self._state_pub = self.create_publisher(String, '/voice/asr_state', 10)
        session_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/voice/session_active', self._on_session, session_qos)
        self.create_subscription(Bool, '/voice/speaking', self._on_speaking, 10)
        self.create_timer(1.0, self._republish_state)
        self._capture_worker = threading.Thread(target=self._capture_loop, daemon=True)
        self._decode_worker = threading.Thread(target=self._decode_loop, daemon=True)
        self._capture_worker.start()
        self._decode_worker.start()
        self._publish_state('IDLE')
        self.get_logger().info('连续收音与 Silero VAD 已启动；Qwen3-ASR 已加载')

    def _load_recognizer(self):
        import sherpa_onnx

        directory = Path(self.get_parameter('qwen3_model_dir').value)
        required = (directory / 'conv_frontend.onnx', directory / 'encoder.int8.onnx',
                    directory / 'decoder.int8.onnx', directory / 'tokenizer/vocab.json',
                    directory / 'tokenizer/merges.txt')
        if not all(path.is_file() for path in required):
            raise RuntimeError(f'Qwen3-ASR 模型文件缺失: {directory}')
        return sherpa_onnx.OfflineRecognizer.from_qwen3_asr(
            conv_frontend=str(required[0]), encoder=str(required[1]),
            decoder=str(required[2]), tokenizer=str(directory / 'tokenizer'),
            num_threads=int(self.get_parameter('num_threads').value),
            max_new_tokens=128, provider='cpu')

    def _load_vad(self):
        import sherpa_onnx

        model = Path(self.get_parameter('vad_model').value)
        if not model.is_file():
            raise RuntimeError(f'VAD 模型文件缺失: {model}')
        config = sherpa_onnx.VadModelConfig()
        config.silero_vad.model = str(model)
        config.silero_vad.threshold = float(self.get_parameter('vad_threshold').value)
        config.silero_vad.min_speech_duration = float(self.get_parameter('vad_min_speech_s').value)
        config.silero_vad.min_silence_duration = float(self.get_parameter('vad_silence_s').value)
        config.silero_vad.max_speech_duration = float(self.get_parameter('vad_max_speech_s').value)
        config.sample_rate = int(self.get_parameter('sample_rate').value)
        config.num_threads = 1
        return sherpa_onnx.VoiceActivityDetector(config, buffer_size_in_seconds=10)

    def _publish_state(self, value):
        self._state_value = str(value)
        self._republish_state()

    def _republish_state(self):
        self._state_pub.publish(String(data=self._state_value))

    def _on_session(self, message):
        active = bool(message.data)
        with self._lock:
            if active == self._active:
                return
            self._active = active
            self._epoch += 1
        if not active:
            self._discard_jobs()
        self._publish_state('LISTENING' if active else 'IDLE')

    def _on_speaking(self, message):
        with self._lock:
            self._speaking = bool(message.data)
            if not self._speaking:
                self._mute_until = time.monotonic() + float(
                    self.get_parameter('tts_cooldown_s').value)

    def _discard_jobs(self):
        while True:
            try:
                self._jobs.get_nowait()
                self._jobs.task_done()
            except queue.Empty:
                return

    def _capture_loop(self):
        sample_rate = int(self.get_parameter('sample_rate').value)
        frame_ms = int(self.get_parameter('frame_ms').value)
        frame_bytes = sample_rate * 2 * frame_ms // 1000
        if sample_rate != 16000 or frame_ms <= 0 or frame_bytes <= 0:
            self._publish_state('ERROR: VAD 需要 16 kHz 有效音频帧')
            return
        while not self._stop.is_set():
            capture = ArecordCapture(str(self.get_parameter('capture_device').value), sample_rate)
            processing = False
            try:
                with self._lock:
                    self._capture = capture
                capture.start()
                while not self._stop.is_set():
                    pcm = capture.read(frame_bytes)
                    now = time.monotonic()
                    with self._lock:
                        active = self._active
                        speaking = self._speaking
                        mute_until = self._mute_until
                        epoch = self._epoch
                    allowed = active and not speaking and now >= mute_until
                    if not allowed:
                        if processing:
                            self._segments.reset()
                            processing = False
                        continue
                    if not processing:
                        self._segments.reset()
                        processing = True
                    samples = np.frombuffer(pcm, dtype='<i2').astype(np.float32) / 32768.0
                    for segment in self._segments.accept(samples):
                        if segment.size == 0:
                            continue
                        job = (epoch, now, segment)
                        try:
                            self._jobs.put_nowait(job)
                        except queue.Full:
                            with self._lock:
                                self._active = False
                                self._epoch += 1
                            self._discard_jobs()
                            self._publish_state('OVERLOAD')
                            self.get_logger().error('ASR 语音队列已满，停止当前语音会话')
                            processing = False
                            self._segments.reset()
                            break
            except Exception as exc:
                if not self._stop.is_set():
                    stack = ' -> '.join(f'{entry.name}:{entry.lineno}'
                                     for entry in traceback.extract_tb(exc.__traceback__))
                    self.get_logger().error(f'持续收音失败: {exc}; {stack}')
                    self._publish_state(f'ERROR: {exc}')
            finally:
                capture.close()
                with self._lock:
                    if self._capture is capture:
                        self._capture = None
                self._segments.reset()
            if not self._stop.is_set():
                self._stop.wait(1.0)

    def _decode_loop(self):
        sample_rate = int(self.get_parameter('sample_rate').value)
        while not self._stop.is_set():
            try:
                epoch, ended_at, samples = self._jobs.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                with self._lock:
                    valid = self._active and epoch == self._epoch
                if not valid:
                    continue
                self._publish_state('RECOGNIZING')
                stream = self._recognizer.create_stream()
                stream.set_option('language', 'Chinese')
                stream.accept_waveform(sample_rate, samples)
                started = time.monotonic()
                self._recognizer.decode_stream(stream)
                text = stream.result.text.strip()
                age = time.monotonic() - ended_at
                with self._lock:
                    valid = self._active and epoch == self._epoch
                if not valid or age > float(self.get_parameter('max_result_age_s').value):
                    self.get_logger().warning(f'丢弃过期或已退出会话的 ASR 结果，age_s={age:.2f}')
                    continue
                self.get_logger().info(
                    f'ASR backend=qwen3 audio_s={samples.size/sample_rate:.2f} '
                    f'decode_s={time.monotonic()-started:.2f} age_s={age:.2f}')
                if text:
                    self._text_pub.publish(String(data=text))
                    self.get_logger().info(f'离线识别结果: {text}')
            except Exception as exc:
                self.get_logger().error(f'Qwen3-ASR 解码失败: {exc}')
                self._publish_state(f'ERROR: {exc}')
            finally:
                self._jobs.task_done()
                with self._lock:
                    active = self._active
                if active and self._jobs.empty() and not self._state_value.startswith('ERROR'):
                    self._publish_state('LISTENING')

    def destroy_node(self):
        self._stop.set()
        with self._lock:
            capture = self._capture
        if capture is not None:
            capture.close()
        self._capture_worker.join(timeout=3)
        self._decode_worker.join(timeout=3)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ContinuousAsrNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
