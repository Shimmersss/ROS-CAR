#!/usr/bin/env python3
"""Plan/download official CPU VAD/KWS models; never starts audio or ROS."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tarfile
import urllib.request

KWS_NAME = 'sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20'
BASE = 'https://github.com/k2-fsa/sherpa-onnx/releases/download/'
ASSETS = {'silero_vad.onnx': BASE + 'asr-models/silero_vad.onnx',
          KWS_NAME + '.tar.bz2': BASE + 'kws-models/' + KWS_NAME + '.tar.bz2'}


def digest(path):
    hasher = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            hasher.update(chunk)
    return hasher.hexdigest()


def prepare(root, download):
    records = []
    for name, url in ASSETS.items():
        path = root / name
        if download:
            root.mkdir(parents=True, exist_ok=True)
            if not path.exists():
                temporary = path.with_suffix(path.suffix + '.part')
                request = urllib.request.Request(url, headers={'User-Agent': 'ROSCAR-voice-lab'})
                with urllib.request.urlopen(request, timeout=60) as response, temporary.open('wb') as stream:
                    shutil.copyfileobj(response, stream)
                temporary.replace(path)
            if name.endswith('.tar.bz2'):
                with tarfile.open(path) as archive:
                    for member in archive.getmembers():
                        rel = Path(member.name)
                        if rel.is_absolute() or '..' in rel.parts or member.issym() or member.islnk():
                            raise ValueError('Unsafe model archive member')
                        if member.isfile():
                            target = root / rel
                            target.parent.mkdir(parents=True, exist_ok=True)
                            with archive.extractfile(member) as source, target.open('wb') as stream:
                                shutil.copyfileobj(source, stream)
        records.append({'url': url, 'path': str(path), 'downloaded': path.exists(),
                        'sha256': digest(path) if path.exists() else None})
    if download:
        files = [root / 'silero_vad.onnx', root / KWS_NAME / 'tokens.txt']
        files.extend(sorted((root / KWS_NAME).glob('*.onnx')))
        if 'matcha-icefall-zh-baker.tar.bz2' in ASSETS:
            files.extend([root / 'matcha-icefall-zh-baker/model-steps-3.onnx',
                          root / 'vocos-22khz-univ.onnx'])
        elif 'vits-melo-tts-zh_en.tar.bz2' in ASSETS:
            files.append(root / 'vits-melo-tts-zh_en/model.onnx')
        (root / 'voice-frontend-manifest.json').write_text(json.dumps(
            {'assets': records, 'files': {str(p.relative_to(root)): digest(p)
                                         for p in files}}, indent=2) + '\n')
    return records


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--download', action='store_true')
    parser.add_argument('--include-asr', action='store_true', help='Also prepare the existing Qwen3-ASR 0.6B INT8 baseline')
    parser.add_argument('--include-tts', action='store_true', help='Also prepare the default Matcha Chinese female TTS and Vocos vocoder')
    parser.add_argument('--tts-model', choices=['matcha', 'melo'], default='matcha',
                        help='TTS assets to prepare; melo is only for historical synthetic fixtures')
    args = parser.parse_args()
    if args.include_asr:
        name = 'sherpa-onnx-qwen3-asr-0.6B-int8-2026-03-25.tar.bz2'
        ASSETS[name] = BASE + 'asr-models/' + name
    if args.include_tts:
        name = ('matcha-icefall-zh-baker.tar.bz2' if args.tts_model == 'matcha'
                else 'vits-melo-tts-zh_en.tar.bz2')
        ASSETS[name] = BASE + 'tts-models/' + name
        if args.tts_model == 'matcha':
            ASSETS['vocos-22khz-univ.onnx'] = BASE + 'vocoder-models/vocos-22khz-univ.onnx'
    print(json.dumps(prepare(args.root, args.download), indent=2))
