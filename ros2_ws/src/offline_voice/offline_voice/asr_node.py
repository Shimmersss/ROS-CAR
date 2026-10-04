"""Wake-triggered, entirely local ASR with selectable backends."""

from collections import deque
import os
from pathlib import Path
import time

import numpy as np
import rclpy
from std_msgs.msg import String

from xfyun_speech.asr_node import XfyunAsrNode
from xfyun_speech.audio import pcm_rms_s16le
from .frontend import NeuralEndpoint, qwen_hotword_options


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
        self.declare_parameter('vad_backend', 'silero')
        self.declare_parameter('vad_model', str(DEFAULT_MODEL_ROOT / 'silero_vad.onnx'))
        self.declare_parameter('endpoint_silence_s', 0.6)
        self.declare_parameter('hotwords_enabled', False)
        self.declare_parameter('robot_hotwords', ['前进', '后退', '左转', '右转', '停止', '停下', '急停', '别动', '跟随', '启动', '状态', '电量', '一下', '一点', '稍微'])
        self._endpoint = None
        if self.get_parameter('vad_backend').value == 'silero':
            self._endpoint = NeuralEndpoint(self.get_parameter('vad_model').value,
                float(self.get_parameter('endpoint_silence_s').value), max_speech_s=float(self.get_parameter('max_utterance_sec').value))
        elif self.get_parameter('vad_backend').value != 'rms':
            raise ValueError('vad_backend must be silero or explicit rms fallback')
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
                max_new_tokens=128, provider='cpu', **qwen_hotword_options(
                    sherpa_onnx.OfflineRecognizer.from_qwen3_asr,
                    self.get_parameter('robot_hotwords').value,
                    self.get_parameter('hotwords_enabled').value))
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
        if self._endpoint is not None:
            return self._listen_neural(trigger_source)
        return self._listen_rms(trigger_source)

    def _listen_rms(self, trigger_source):
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
        capture = self._capture(device, sample_rate)
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
            while not self._stop.is_set() and not self._cancel.is_set():
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
                        self._publish_state('SPEECH')
                else:
                    frames.append(pcm)
                    silence_ms = 0 if rms >= threshold else silence_ms + frame_ms
                now = time.monotonic()
                # Do not cut a word whose onset straddles the waiting deadline.
                if not speech_started and not candidate_ms and now - started >= speech_timeout:
                    return
                if speech_started and now - speech_at >= max_utterance:
                    return  # Reject an incomplete utterance at the hard limit.
                if speech_started and silence_ms >= silence_limit:
                    break
            if self._stop.is_set() or self._cancel.is_set() or not speech_started:
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
            if self._speaking or self._stop.is_set() or self._cancel.is_set():
                return
            self.get_logger().info(
                f'ASR backend={self._backend} audio_s={len(samples)/sample_rate:.2f} '
                f'decode_s={time.monotonic()-decode_at:.2f}')
            if text:
                self._text_pub.publish(String(data=text))
                self.get_logger().info(f'离线识别结果: {text}')
        finally:
            capture.close()


    def _listen_neural(self, trigger_source):
        del trigger_source
        endpoint = self._endpoint
        endpoint.reset()
        capture = self._capture(str(self.get_parameter('capture_device').value), 16000)
        started, speech_at, audio = time.monotonic(), None, None
        self._phase = 'LISTENING'
        try:
            capture.start()
            self._publish_state('LISTENING')
            while not self._stop.is_set() and not self._cancel.is_set():
                pcm = capture.read(1280)
                if self._speaking:
                    return
                audio = endpoint.accept(np.frombuffer(pcm, dtype='<i2').astype(np.float32) / 32768.)
                now = time.monotonic()
                if endpoint.speech_started and speech_at is None:
                    speech_at = now
                    self._publish_state('SPEECH')
                if audio is not None:
                    break
                if speech_at is None and now - started >= float(self.get_parameter('speech_timeout_sec').value):
                    return
                if speech_at is not None and now - speech_at >= float(self.get_parameter('max_utterance_sec').value):
                    return
            if audio is None or self._cancel.is_set() or self._stop.is_set():
                return
            capture.close()
            self._phase = 'RECOGNIZING'
            self._publish_state('RECOGNIZING')
            stream = self._recognizer.create_stream()
            if self._backend == 'qwen3':
                stream.set_option('language', 'Chinese')
            stream.accept_waveform(16000, audio)
            if self._backend in ('qwen3', 'fire_red_ctc', 'sensevoice'):
                self._recognizer.decode_stream(stream)
                text = stream.result.text.strip()
            else:
                stream.accept_waveform(16000, np.zeros(10560, dtype=np.float32))
                stream.input_finished()
                while self._recognizer.is_ready(stream):
                    self._recognizer.decode_stream(stream)
                text = self._recognizer.get_result(stream).strip()
            if text and not (self._cancel.is_set() or self._speaking or self._stop.is_set()):
                self._text_pub.publish(String(data=text))
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
