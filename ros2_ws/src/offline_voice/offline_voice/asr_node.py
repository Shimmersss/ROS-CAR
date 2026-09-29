"""Wake-triggered, entirely local streaming ASR."""

import os
from pathlib import Path
import time

import numpy as np
import rclpy
from std_msgs.msg import String

from xfyun_speech.asr_node import XfyunAsrNode
from xfyun_speech.audio import ArecordCapture, pcm_rms_s16le


DEFAULT_MODEL_ROOT = Path(os.environ.get(
    'ROSCAR_OFFLINE_MODEL_ROOT', '/home/wheeltec/ROSCAR-offline/models'))


class OfflineAsrNode(XfyunAsrNode):
    def __init__(self):
        super().__init__()
        self.declare_parameter('model_dir', str(
            DEFAULT_MODEL_ROOT / 'sherpa-onnx-streaming-paraformer-bilingual-zh-en'))
        self.declare_parameter('num_threads', 2)
        self._recognizer = self._load_recognizer()
        self.get_logger().info('离线 ASR 模型已加载')

    def _load_recognizer(self):
        import sherpa_onnx

        model_dir = Path(self.get_parameter('model_dir').value)
        names = ('tokens.txt', 'encoder.int8.onnx', 'decoder.int8.onnx')
        missing = [name for name in names if not (model_dir / name).is_file()]
        if missing:
            raise RuntimeError(f'离线 ASR 模型文件缺失: {model_dir}: {missing}')
        return sherpa_onnx.OnlineRecognizer.from_paraformer(
            tokens=str(model_dir / names[0]),
            encoder=str(model_dir / names[1]),
            decoder=str(model_dir / names[2]),
            num_threads=int(self.get_parameter('num_threads').value),
            sample_rate=int(self.get_parameter('sample_rate').value),
            provider='cpu',
        )

    def _listen_once(self, trigger_source):
        del trigger_source
        sample_rate = int(self.get_parameter('sample_rate').value)
        frame_ms = int(self.get_parameter('frame_ms').value)
        frame_bytes = sample_rate * 2 * frame_ms // 1000
        threshold = float(self.get_parameter('energy_threshold').value)
        onset_ms = int(self.get_parameter('speech_onset_ms').value)
        silence_limit = int(self.get_parameter('silence_ms').value)
        speech_timeout = float(self.get_parameter('speech_timeout_sec').value)
        max_utterance = float(self.get_parameter('max_utterance_sec').value)
        device = str(self.get_parameter('capture_device').value)
        capture = ArecordCapture(device, sample_rate)
        stream = self._recognizer.create_stream()
        speech_started = False
        candidate_ms = 0
        silence_ms = 0
        self._phase = 'LISTENING'
        try:
            capture.start()
            self._publish_state('LISTENING')
            started = time.monotonic()
            while not self._stop.is_set() and not self._speaking:
                pcm = capture.read(frame_bytes)
                rms = pcm_rms_s16le(pcm)
                if rms >= threshold:
                    candidate_ms += frame_ms
                    if candidate_ms >= onset_ms:
                        speech_started = True
                    if speech_started:
                        silence_ms = 0
                elif speech_started:
                    silence_ms += frame_ms
                else:
                    candidate_ms = 0

                samples = np.frombuffer(pcm, dtype='<i2').astype(np.float32) / 32768.0
                stream.accept_waveform(sample_rate, samples)
                while self._recognizer.is_ready(stream):
                    self._recognizer.decode_stream(stream)
                elapsed = time.monotonic() - started
                if not speech_started and elapsed >= speech_timeout:
                    break
                if speech_started and silence_ms >= silence_limit:
                    break
                if elapsed >= max_utterance:
                    break

            self._phase = 'RECOGNIZING'
            self._publish_state('RECOGNIZING')
            stream.accept_waveform(sample_rate, np.zeros(int(sample_rate * .66), dtype=np.float32))
            stream.input_finished()
            while self._recognizer.is_ready(stream):
                self._recognizer.decode_stream(stream)
            text = self._recognizer.get_result(stream).strip()
            # Noise or an empty wake event must never become a vehicle command.
            if speech_started and text:
                self._text_pub.publish(String(data=text))
                self.get_logger().info(f'离线识别结果: {text}')
            else:
                self.get_logger().info('未检测到有效语音或识别结果为空')
        finally:
            capture.close()


def main(args=None):
    rclpy.init(args=args)
    node = OfflineAsrNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
