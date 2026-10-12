#!/usr/bin/env python3
"""Generate explicitly synthetic MeloTTS WAVs for smoke/regression tests.

These are digital TTS samples, not a real-user or real-car benchmark.
"""
import argparse
import json
from pathlib import Path
import wave
import numpy as np
import sherpa_onnx

CASES = [('stop', '停止', 'STOP'), ('halt', '停下', 'STOP'), ('estop', '急停', 'ESTOP'),
         ('stay', '别动', 'STOP'), ('forward', '前进', 'MOVE:FORWARD'),
         ('reverse', '后退一秒', 'MOVE:REVERSE'), ('micro_left', '左转一点', 'TURN:LEFT'),
         ('negated', '不要前进', 'NONE'), ('question', '怎么让小车前进', 'NONE'),
         ('feedback', '已唤醒，请说指令', 'NONE')]


def prepare(root, output):
    cfg = sherpa_onnx.OfflineTtsConfig(model=sherpa_onnx.OfflineTtsModelConfig(
        vits=sherpa_onnx.OfflineTtsVitsModelConfig(model=str(root / 'model.onnx'),
            lexicon=str(root / 'lexicon.txt'), tokens=str(root / 'tokens.txt')),
        provider='cpu', num_threads=2), rule_fsts=','.join(str(root / p) for p in ('date.fst', 'number.fst')))
    if not cfg.validate():
        raise ValueError('MeloTTS configuration invalid')
    tts = sherpa_onnx.OfflineTts(cfg)
    output.mkdir(parents=True, exist_ok=True)
    rows = []
    for sample_id, text, intent in CASES:
        audio = tts.generate(text, sherpa_onnx.GenerationConfig())
        # Explicit interpolation for a digital fixture, never presented as the
        # physical microphone's sample conversion or a perceptual quality test.
        count = round(len(audio.samples) * 16000 / audio.sample_rate)
        samples = np.interp(np.arange(count) / 16000,
                            np.arange(len(audio.samples)) / audio.sample_rate, audio.samples)
        samples = np.concatenate((np.zeros(8000), samples, np.zeros(16000)))
        path = output / (sample_id + '.wav')
        with wave.open(str(path), 'wb') as stream:
            stream.setnchannels(1); stream.setsampwidth(2); stream.setframerate(16000)
            stream.writeframes((np.clip(samples, -1, 1) * 32767).astype('<i2').tobytes())
        rows.append(dict(id=sample_id, wav=path.name, reference_text=text, expected_intent=intent,
                         reference_keywords=[text] if sample_id in ('stop', 'halt', 'estop', 'stay') else [],
                         sample_type='synthetic_tts', condition='MeloTTS_digital_resampled_no_acoustic_path',
                         speech_start_s=None, speech_end_s=None, stop_word_end_s=None,
                         annotation_status='Synthetic input text known; acoustic word boundaries not annotated'))
    manifest = output / 'manifest.jsonl'
    manifest.write_text(''.join(json.dumps(row, ensure_ascii=False) + '\n' for row in rows))
    return manifest


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(prepare(args.model, args.output))
