"""Wake-triggered, entirely local ASR with selectable backends."""

from collections import deque
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
        self.declare_parameter('asr_backend', 'paraformer')
        self.declare_parameter('sensevoice_model_dir', str(DEFAULT_MODEL_ROOT /
            'sherpa-onnx-sense-voice-zh-en-ja-ko-yue-int8-2024-07-17'))
        self.declare_parameter('fire_red_model_dir', str(DEFAULT_MODEL_ROOT /
            'sherpa-onnx-fire-red-asr2-ctc-zh_en-int8-2026-02-25'))
        self.declare_parameter('qwen3_model_dir', str(DEFAULT_MODEL_ROOT /
            'sherpa-onnx-qwen3-asr-0.6B-int8-2026-03-25'))
        self.declare_parameter('num_threads', 2)
        self._recognizer = self._load_recognizer()
        self.get_logger().info(f'离线 ASR 模型已加载: {self._backend}')

    def _load_recognizer(self):
        import sherpa_onnx

        self._backend = str(self.get_parameter('asr_backend').value)
        if self._backend == 'qwen3':
            directory = Path(self.get_parameter('qwen3_model_dir').value)
            required = (directory / 'conv_frontend.onnx',
                        directory / 'encoder.int8.onnx',
                        directory / 'decoder.int8.onnx',
                        directory / 'tokenizer/vocab.json',
                        directory / 'tokenizer/merges.txt')
            if not all(path.is_file() for path in required):
                raise RuntimeError(f'Qwen3-ASR 模型文件缺失: {directory}')
            return sherpa_onnx.OfflineRecognizer.from_qwen3_asr(
                conv_frontend=str(required[0]), encoder=str(required[1]),
                decoder=str(required[2]), tokenizer=str(directory / 'tokenizer'),
                num_threads=int(self.get_parameter('num_threads').value),
                max_new_tokens=128, provider='cpu')
        if self._backend == 'fire_red_ctc':
            directory = Path(self.get_parameter('fire_red_model_dir').value)
            required = (directory / 'model.int8.onnx', directory / 'tokens.txt')
            if not all(path.is_file() for path in required):
                raise RuntimeError(f'FireRedASR2 模型文件缺失: {directory}')
            return sherpa_onnx.OfflineRecognizer.from_fire_red_asr_ctc(
                model=str(required[0]), tokens=str(required[1]),
                num_threads=int(self.get_parameter('num_threads').value),
                provider='cpu')
        if self._backend == 'sensevoice':
            directory = Path(self.get_parameter('sensevoice_model_dir').value)
            return sherpa_onnx.OfflineRecognizer.from_sense_voice(
                model=str(directory / 'model.int8.onnx'),
                tokens=str(directory / 'tokens.txt'), language='zh', use_itn=True,
                num_threads=int(self.get_parameter('num_threads').value), provider='cpu')
        if self._backend != 'paraformer':
            raise ValueError(f'Unsupported ASR backend: {self._backend}')
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
        # Keep only a short pre-roll before onset, not an entire noisy wait.
        preroll = deque(maxlen=max(1, 400 // frame_ms))
        frames = []
        speech_started = False
        candidate_ms = silence_ms = 0
        self._phase = 'LISTENING'
        try:
            capture.start()
            self._publish_state('LISTENING')
            started = time.monotonic()
            speech_at = None
            while not self._stop.is_set():
                pcm = capture.read(frame_bytes)
                if self._speaking:
                    self.get_logger().info('收音被播报打断，丢弃本轮音频')
                    return
                rms = pcm_rms_s16le(pcm)
                if not speech_started:
                    preroll.append(pcm)
                    candidate_ms = candidate_ms + frame_ms if rms >= threshold else 0
                    if candidate_ms >= onset_ms:
                        speech_started = True
                        speech_at = time.monotonic()
                        frames.extend(preroll)
                else:
                    frames.append(pcm)
                    silence_ms = 0 if rms >= threshold else silence_ms + frame_ms
                now = time.monotonic()
                # Do not cut a word whose onset straddles the waiting deadline.
                if not speech_started and not candidate_ms and now - started >= speech_timeout:
                    return
                if speech_started and (silence_ms >= silence_limit or now - speech_at >= max_utterance):
                    break
            if self._stop.is_set() or not speech_started:
                return
            self._phase = 'RECOGNIZING'
            self._publish_state('RECOGNIZING')
            samples = np.frombuffer(b''.join(frames), dtype='<i2').astype(np.float32) / 32768.0
            stream = self._recognizer.create_stream()
            decode_at = time.monotonic()
            if self._backend == 'qwen3':
                stream.set_option('language', 'Chinese')
            stream.accept_waveform(sample_rate, samples)
            if self._backend in ('sensevoice', 'fire_red_ctc', 'qwen3'):
                self._recognizer.decode_stream(stream)
                text = stream.result.text.strip()
            else:
                stream.accept_waveform(sample_rate, np.zeros(int(sample_rate * .66), dtype=np.float32))
                stream.input_finished()
                while self._recognizer.is_ready(stream):
                    self._recognizer.decode_stream(stream)
                text = self._recognizer.get_result(stream).strip()
            if self._speaking or self._stop.is_set():
                return
            self.get_logger().info(
                f'ASR backend={self._backend} audio_s={len(samples)/sample_rate:.2f} '
                f'decode_s={time.monotonic()-decode_at:.2f}')
            if text:
                self._text_pub.publish(String(data=text))
                self.get_logger().info(f'离线识别结果: {text}')
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
