"""Local Chinese TTS while retaining the established ALSA/speaking topics."""

import os
from pathlib import Path

import numpy as np
import rclpy

from xfyun_speech.audio import AplaySink
from xfyun_speech.tts_node import XfyunTtsNode


DEFAULT_MODEL_ROOT = Path(os.environ.get(
    'ROSCAR_OFFLINE_MODEL_ROOT', '/home/wheeltec/ROSCAR-offline/models'))


class OfflineTtsNode(XfyunTtsNode):
    def __init__(self):
        super().__init__()
        self.declare_parameter('model_dir', str(DEFAULT_MODEL_ROOT / 'vits-melo-tts-zh_en'))
        self.declare_parameter('num_threads', 2)
        self.declare_parameter('speech_speed', 1.0)
        self._tts = self._load_tts()
        self.get_logger().info('离线 TTS 模型已加载')

    def _load_tts(self):
        import sherpa_onnx

        model_dir = Path(self.get_parameter('model_dir').value)
        names = ('model.onnx', 'lexicon.txt', 'tokens.txt', 'date.fst', 'number.fst')
        missing = [name for name in names if not (model_dir / name).is_file()]
        if missing:
            raise RuntimeError(f'离线 TTS 模型文件缺失: {model_dir}: {missing}')
        config = sherpa_onnx.OfflineTtsConfig(
            model=sherpa_onnx.OfflineTtsModelConfig(
                vits=sherpa_onnx.OfflineTtsVitsModelConfig(
                    model=str(model_dir / names[0]),
                    lexicon=str(model_dir / names[1]),
                    tokens=str(model_dir / names[2]),
                ),
                provider='cpu',
                num_threads=int(self.get_parameter('num_threads').value),
            ),
            rule_fsts=','.join(str(model_dir / name) for name in names[3:]),
        )
        if not config.validate():
            raise RuntimeError('离线 TTS 模型配置无效')
        return sherpa_onnx.OfflineTts(config)

    def _speak(self, text):
        import sherpa_onnx

        generation = sherpa_onnx.GenerationConfig()
        generation.speed = float(self.get_parameter('speech_speed').value)
        self._publish_speaking(True)
        self._publish_state('SPEAKING')
        audio = self._tts.generate(text, generation)
        if len(audio.samples) == 0:
            raise RuntimeError('离线 TTS 未生成音频')
        pcm = (np.clip(audio.samples, -1.0, 1.0) * 32767).astype('<i2').tobytes()
        sink = AplaySink(str(self.get_parameter('playback_device').value), audio.sample_rate)
        sink.start()
        try:
            sink.write(pcm)
        finally:
            sink.close()


def main(args=None):
    rclpy.init(args=args)
    node = OfflineTtsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
