#!/usr/bin/env python3
"""Offline WAV -> VAD/KWS/optional Qwen hotword A/B; never opens ALSA or ROS."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import platform
import re
import sys
import time
import wave
import numpy as np

PROJECT = Path(__file__).resolve().parents[1]
for package in ('offline_voice', 'voice_command_router'):
    sys.path.insert(0, str(PROJECT / 'ros2_ws/src' / package))
from offline_voice.frontend import NeuralEndpoint, EmergencySpotter, qwen_hotword_options
from voice_command_router.validation import parse_intent, CommandValidationError


def read_wav(path):
    with wave.open(str(path)) as stream:
        if (stream.getframerate(), stream.getnchannels(), stream.getsampwidth()) != (16000, 1, 2):
            raise ValueError(f'{path}: require 16 kHz mono PCM16 WAV; convert explicitly')
        samples = np.frombuffer(stream.readframes(stream.getnframes()), dtype='<i2').astype(np.float32) / 32768.
    if not 0 < len(samples) <= 16000 * 60:
        raise ValueError('Use WAVs of 0..60 seconds, one utterance per sample')
    return samples


def normalize(text):
    return re.sub(r'[^\w]', '', str(text).lower())


def edit_distance(reference, prediction):
    row = list(range(len(prediction) + 1))
    for i, char in enumerate(reference, 1):
        next_row = [i]
        for j, other in enumerate(prediction, 1):
            next_row.append(min(row[j] + 1, next_row[-1] + 1, row[j-1] + (char != other)))
        row = next_row
    return row[-1]


def percentile(values):
    return {'n': len(values), 'p50_ms': float(np.percentile(values, 50)) if values else None,
            'p95_ms': float(np.percentile(values, 95)) if values else None}


def summarize(rows):
    refs = [r for r in rows if r.get('reference_text') is not None and r.get('asr_text') is not None]
    annotated_boundaries = [r for r in rows if r.get('speech_start_s') is not None and r.get('speech_end_s') is not None]
    truncations = sum(r['endpoint_audio_s'] is None or r.get('capture_start_s', math.inf) > r['speech_start_s'] + .04 or r.get('capture_end_s', -math.inf) < r['speech_end_s'] - .04 for r in annotated_boundaries)
    chars = sum(len(normalize(r['reference_text'])) for r in refs)
    errors = sum(edit_distance(normalize(r['reference_text']), normalize(r['asr_text'])) for r in refs)
    commands = [r for r in rows if r.get('expected_intent') is not None and r.get('asr_text') is not None]
    negatives = [r for r in commands if r['expected_intent'] == 'NONE']
    dangerous = {'MOVE', 'TURN', 'ARM', 'SET_MODE'}
    keyword_pairs = [(word, r['asr_text']) for r in refs for word in r.get('reference_keywords', [])]
    annotated_stops = [r for r in rows if r.get('stop_word_end_s') is not None]
    detected_stops = [r for r in annotated_stops if r['kws_events']]
    return {'cer': errors / chars if chars else None,
            'critical_keyword_accuracy': sum(normalize(word) in normalize(text) for word, text in keyword_pairs) / len(keyword_pairs) if keyword_pairs else None,
            'critical_keywords': len(keyword_pairs), 'cer_characters': chars,
            'command_intent_accuracy': sum(r['intent'] == r['expected_intent'] for r in commands) / len(commands) if commands else None,
            'command_samples': len(commands), 'physical_command_success_rate': None,
            'vad_truncation_rate': truncations / len(annotated_boundaries) if annotated_boundaries else None,
            'vad_boundary_samples': len(annotated_boundaries),
            'offline_text_ready_latency': percentile([(r['endpoint_audio_s'] - r['speech_end_s'] + r['decode_wall_s']) * 1000 for r in rows if r.get('speech_end_s') is not None and r.get('decode_wall_s') is not None]),
            'normal_ros_command_start_latency': None,
            'dangerous_false_intent_rate': sum(r['intent'].split(':')[0] in dangerous for r in negatives) / len(negatives) if negatives else None,
            'negative_samples': len(negatives), 'kws_stop_recall': len(detected_stops) / len(annotated_stops) if annotated_stops else None,
            'kws_annotated_stops': len(annotated_stops),
            'kws_latency_from_word_end': percentile([(r['kws_events'][0]['audio_time_s'] - r['stop_word_end_s']) * 1000 for r in detected_stops]),
            'vad_endpoint_latency': percentile([(r['endpoint_audio_s'] - r['speech_end_s']) * 1000 for r in rows if r.get('speech_end_s') is not None and r['endpoint_audio_s'] is not None]),
            'decode_wall_latency': percentile([r['decode_wall_s'] * 1000 for r in rows if r.get('decode_wall_s') is not None]),
            'kws_false_stop_samples': sum(bool(r['kws_events']) for r in rows if r.get('sample_type') in ('noise', 'robot_tts', 'negative')),
            'note': 'Intent accuracy is a text-layer metric. No chassis or actuator was used. Missing labels/measurements remain null.'}


def smoke_manifest(output):
    output.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(42)
    records = []
    for name, samples in [('silence', np.zeros(16000 * 5)), ('synthetic_noise', rng.normal(0, .015, 16000 * 5))]:
        path = output / (name + '.wav')
        with wave.open(str(path), 'wb') as stream:
            stream.setnchannels(1); stream.setsampwidth(2); stream.setframerate(16000)
            stream.writeframes((np.clip(samples, -1, 1) * 32767).astype('<i2').tobytes())
        records.append(dict(id=name, wav=path.name, sample_type='noise', reference_text='', expected_intent='NONE', condition='synthetic_local_smoke'))
    vendor = PROJECT / 'ros2_ws/src/wheeltec_mic/wheeltec_mic_ros2/feedback_voice'
    for name in ('stop', 'turn_left', 'turn_right', 'voice_control'):
        records.append(dict(id='vendor_' + name, wav=str(vendor / (name + '.wav')), sample_type='robot_tts', reference_text=None, expected_intent=None, condition='prerecorded_vendor_feedback_not_real_car'))
    manifest = output / 'smoke.jsonl'
    manifest.write_text(''.join(json.dumps(r, ensure_ascii=False) + '\n' for r in records))
    return manifest


def qwen(root, hotwords, enabled):
    import sherpa_onnx
    factory = sherpa_onnx.OfflineRecognizer.from_qwen3_asr
    return factory(conv_frontend=str(root / 'conv_frontend.onnx'), encoder=str(root / 'encoder.int8.onnx'),
                   decoder=str(root / 'decoder.int8.onnx'), tokenizer=str(root / 'tokenizer'),
                   num_threads=2, provider='cpu', max_new_tokens=128,
                   **qwen_hotword_options(factory, hotwords, enabled))


def benchmark(args):
    import sherpa_onnx
    manifest = args.manifest.resolve()
    records = [json.loads(line) for line in manifest.read_text().splitlines() if line.strip()]
    if not records or len({r['id'] for r in records}) != len(records):
        raise ValueError('Manifest must be nonempty with unique sample IDs')
    modes = (False, True) if args.qwen_model else (False,)
    summaries, all_rows = {}, []
    keywords = ['前进', '后退', '左转', '右转', '停止', '停下', '急停', '别动', '跟随', '一下', '稍微']
    kws = EmergencySpotter(args.models / 'sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20', PROJECT / 'ros2_ws/src/offline_voice/config/emergency_keywords.txt')
    for hotword_mode in modes:
        recognizer = qwen(args.qwen_model, keywords, hotword_mode) if args.qwen_model else None
        for silence_s in (.3, .4, .5, .6, .7, .8):
            endpoint = NeuralEndpoint(args.models / 'silero_vad.onnx', silence_s)
            rows = []
            for record in records:
                path = (manifest.parent / record['wav']).resolve()
                samples = read_wav(path)
                endpoint.reset(); kws.reset()
                endpoint_audio, endpoint_at, events = None, None, []
                # Tail silence permits endpoint/KWS finalization. Times are replay
                # audio coordinates, not measured microphone/ROS/brake latency.
                padded = np.concatenate((samples, np.zeros(16000, dtype=np.float32)))
                tick_cost = []
                for offset in range(0, len(padded), 640):
                    chunk = padded[offset:offset + 640]
                    if len(chunk) < 640:
                        chunk = np.pad(chunk, (0, 640 - len(chunk)))
                    started = time.perf_counter()
                    keyword = kws.accept(chunk)
                    tick_cost.append((time.perf_counter() - started) * 1000)
                    audio = endpoint.accept(chunk) if endpoint_audio is None else None
                    if keyword:
                        events.append({'keyword': keyword, 'audio_time_s': (offset + 640) / 16000})
                    if audio is not None:
                        endpoint_audio, endpoint_at = audio, (offset + 640) / 16000
                text, decode_s = None, None
                if recognizer:
                    text = ''
                    if endpoint_audio is not None:
                        stream = recognizer.create_stream(); stream.set_option('language', 'Chinese')
                        stream.accept_waveform(16000, endpoint_audio)
                        started = time.perf_counter(); recognizer.decode_stream(stream)
                        decode_s, text = time.perf_counter() - started, stream.result.text.strip()
                try:
                    parsed = parse_intent(text) if text else None
                    intent = parsed['action'] if parsed else 'NONE'
                    if parsed and (parsed.get('direction') or parsed.get('mode')):
                        intent += ':' + (parsed.get('direction') or parsed['mode'])
                except CommandValidationError:
                    intent = 'NONE'
                row = dict(record, wav=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                           endpoint_silence_s=silence_s, hotwords=hotword_mode, endpoint_audio_s=endpoint_at,
                           endpoint_sample_count=len(endpoint_audio) if endpoint_audio is not None else 0,
                           kws_events=events, kws_compute_latency=percentile(tick_cost),
                           asr_text=text, intent=intent, decode_wall_s=decode_s)
                if endpoint.last_segment:
                    row.update(endpoint.last_segment)
                rows.append(row)
            summaries[f'hotwords={hotword_mode},endpoint={silence_s}'] = summarize(rows)
            all_rows.extend(rows)
    return dict(platform=platform.platform(), sherpa_onnx_version=sherpa_onnx.__version__,
                manifest_sha256=hashlib.sha256(manifest.read_bytes()).hexdigest(),
                evidence_scope='Local CPU prerecorded/synthetic WAV replay; no real microphone, TTS overlap or car',
                summaries=summaries, rows=all_rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--models', type=Path)
    parser.add_argument('--manifest', type=Path)
    parser.add_argument('--qwen-model', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--prepare-smoke', type=Path)
    args = parser.parse_args()
    if args.prepare_smoke:
        print(smoke_manifest(args.prepare_smoke)); return
    if not all((args.models, args.manifest, args.output)):
        parser.error('--models, --manifest and --output are required for benchmarking')
    result = benchmark(args)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    print(json.dumps({'output': str(args.output), 'summaries': result['summaries']}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
