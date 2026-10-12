"""Model adapters shared by ROS nodes and the hardware-free WAV benchmark."""
from pathlib import Path
import inspect
import numpy as np


class NeuralEndpoint:
    def __init__(self, model, silence_s=0.6, preroll_s=0.4, max_speech_s=15):
        import sherpa_onnx
        if not Path(model).is_file():
            raise ValueError('Silero VAD model is missing')
        if silence_s not in (0.3, 0.4, 0.5, 0.6, 0.7, 0.8):
            raise ValueError('Select a tested endpoint grid value: 0.3..0.8 seconds')
        config = sherpa_onnx.VadModelConfig()
        config.silero_vad.model = str(model)
        config.silero_vad.min_silence_duration = silence_s
        config.silero_vad.min_speech_duration = 0.08
        config.silero_vad.max_speech_duration = max_speech_s
        config.sample_rate, config.num_threads, config.provider = 16000, 1, 'cpu'
        self.vad = sherpa_onnx.VoiceActivityDetector(config, buffer_size_in_seconds=60)
        self.preroll = int(preroll_s * 16000)
        self.max_speech_s = max_speech_s
        self.reset()

    def reset(self):
        self.vad.reset()
        self.frames = []
        self.speech_started = False
        self.last_segment = None

    def accept(self, samples):
        samples = np.asarray(samples, dtype=np.float32)
        self.frames.append(samples.copy())
        self.vad.accept_waveform(samples)
        self.speech_started |= self.vad.is_speech_detected()
        if self.vad.empty():
            return None
        segment = self.vad.front
        if len(segment.samples) >= self.max_speech_s * 16000:
            raise ValueError('Utterance reached hard limit; partial command rejected')
        start, end = max(0, segment.start - self.preroll), segment.start + len(segment.samples)
        self.last_segment = dict(capture_start_s=start / 16000, capture_end_s=end / 16000)
        output = np.concatenate(self.frames)[start:end]
        self.vad.pop()
        return output


class EmergencySpotter:
    KEYWORDS = {'停止', '停下', '急停', '别动'}

    def __init__(self, model_dir, keywords_file):
        import sherpa_onnx
        root = Path(model_dir)
        suffix = 'epoch-13-avg-2-chunk-8-left-64'
        paths = dict(encoder=root / f'encoder-{suffix}.int8.onnx',
                     decoder=root / f'decoder-{suffix}.onnx',
                     joiner=root / f'joiner-{suffix}.int8.onnx',
                     tokens=root / 'tokens.txt', keywords_file=Path(keywords_file))
        if not all(path.is_file() for path in paths.values()):
            raise ValueError('Emergency KWS model/keywords files are missing')
        # Reject unknown tokens before handing configuration to native code.
        tokens = {line.split()[0] for line in paths['tokens'].read_text().splitlines()}
        labels = set()
        for line in paths['keywords_file'].read_text().splitlines():
            fields = line.split()
            if not fields or not fields[-1].startswith('@') or any(t not in tokens for t in fields[:-1]):
                raise ValueError('Invalid emergency keyword tokens')
            labels.add(fields[-1][1:])
        if labels != self.KEYWORDS:
            raise ValueError('Emergency keyword list must contain exactly the four stop phrases')
        self.spotter = sherpa_onnx.KeywordSpotter(**{k: str(v) for k, v in paths.items()},
            num_threads=1, provider='cpu', sample_rate=16000,
            keywords_score=1.0, keywords_threshold=0.25, num_trailing_blanks=1)
        self.stream = self.spotter.create_stream()

    def accept(self, samples):
        self.stream.accept_waveform(16000, np.asarray(samples, dtype=np.float32))
        while self.spotter.is_ready(self.stream):
            self.spotter.decode_stream(self.stream)
            keyword = self.spotter.get_result(self.stream)
            if keyword in self.KEYWORDS:
                self.spotter.reset_stream(self.stream)
                return keyword
        return None

    def reset(self):
        self.stream = self.spotter.create_stream()


def qwen_hotword_options(factory, hotwords, enabled=False):
    """Use native contextual bias only when explicitly enabled and supported."""
    if not enabled:
        return {}
    if 'hotwords' not in inspect.signature(factory).parameters:
        raise ValueError('This sherpa-onnx build does not support Qwen3-ASR hotwords')
    if not hotwords or len(hotwords) > 20 or any(',' in word or not word for word in hotwords):
        raise ValueError('Use 1..20 nonempty domain hotwords')
    return {'hotwords': ','.join(hotwords)}
