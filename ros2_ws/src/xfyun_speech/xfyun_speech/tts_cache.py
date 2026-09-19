"""Pre-synthesised TTS cache for fixed phrases.

A short fixed reply ("我在") still costs a WebSocket handshake and a round trip
to iFLYTEK, which measured at roughly 0.9 s before the first audio. Synthesising
such a reply once and replaying the PCM removes that entirely.

The file name carries a fingerprint of everything that changes the audio (voice,
speed, pitch, volume, sample rate, text), so switching the voice can never serve
a stale clip. Pure stdlib: no ROS, no network, unit-testable anywhere.
"""

import hashlib
import re
import wave
from pathlib import Path

_SAFE = re.compile(r'[^0-9A-Za-z]+')


def clip_name(text, voice_name, speed, pitch, volume, sample_rate):
    """Stable file name; any change to the audio settings changes the name."""
    fingerprint = '|'.join(str(item) for item in (
        text, voice_name, speed, pitch, volume, sample_rate))
    digest = hashlib.sha256(fingerprint.encode('utf-8')).hexdigest()[:16]
    prefix = _SAFE.sub('', text)[:12] or 'clip'
    return f'{prefix}-{digest}.wav'


def wav_bytes(pcm, sample_rate, channels=1, sample_width=2):
    """Wrap raw little-endian PCM in a WAV container."""
    import io

    buffer = io.BytesIO()
    with wave.open(buffer, 'wb') as handle:
        handle.setnchannels(channels)
        handle.setsampwidth(sample_width)
        handle.setframerate(sample_rate)
        handle.writeframes(pcm)
    return buffer.getvalue()


def cache_dir(configured, voice_name, speed, pitch, volume, sample_rate):
    """Per-setting directory, so auditioning voices never mixes clips up."""
    base = Path(configured).expanduser()
    settings = '-'.join(_SAFE.sub('', str(item)) or 'x' for item in (
        voice_name, speed, pitch, volume, sample_rate))
    return base / settings


def default_cache_path():
    """Where pre-synthesised clips live unless the caller overrides it."""
    return str(Path('~/.cache/roscar/tts').expanduser())
