"""Pure protocol helpers for iFLYTEK IAT and TTS."""

import base64
import re

# Punctuation and spacing carry no meaning for a phrase comparison, and ASR
# output is inconsistent about them.
_NOISE = re.compile(r'[^\u4e00-\u9fffA-Za-z0-9]')


def normalize_phrase(text):
    """Strip punctuation and whitespace so phrases compare by words alone."""
    return _NOISE.sub('', text or '')


def phrase_matches(text, phrases):
    """True when the whole normalised `text` equals one of `phrases`."""
    normalized = normalize_phrase(text)
    if not normalized:
        return False
    return normalized in {normalize_phrase(item) for item in phrases or () if item}


class XfyunResponseError(RuntimeError):
    """Raised when an iFLYTEK response contains a non-zero error code."""


class IatResultAssembler:
    """Assemble incremental and replacement IAT result segments."""

    def __init__(self):
        self._segments = {}

    @property
    def text(self):
        return ''.join(self._segments[key] for key in sorted(self._segments))

    def consume(self, payload):
        code = int(payload.get('code', 0))
        if code != 0:
            message = payload.get('message', 'unknown iFLYTEK error')
            raise XfyunResponseError(f'iFLYTEK IAT error {code}: {message}')

        data = payload.get('data') or {}
        result = data.get('result') or {}
        words = []
        for word_set in result.get('ws') or []:
            candidates = word_set.get('cw') or []
            if candidates:
                words.append(str(candidates[0].get('w', '')))

        if words:
            serial_number = int(result.get('sn', len(self._segments)))
            if result.get('pgs') == 'rpl':
                replace_range = result.get('rg') or []
                if len(replace_range) == 2:
                    first, last = (int(value) for value in replace_range)
                    for key in range(first, last + 1):
                        self._segments.pop(key, None)
            self._segments[serial_number] = ''.join(words)

        return self.text, int(data.get('status', 0)) == 2


def make_iat_frame(app_id, audio, status, *, language='zh_cn', accent='mandarin',
                   sample_rate=16000, vad_eos_ms=1200):
    frame = {
        'data': {
            'status': int(status),
            'format': f'audio/L16;rate={int(sample_rate)}',
            'audio': base64.b64encode(audio).decode('ascii'),
            'encoding': 'raw',
        },
    }
    if status == 0:
        frame['common'] = {'app_id': app_id}
        frame['business'] = {
            'language': language,
            'domain': 'iat',
            'accent': accent,
            'dwa': 'wpgs',
            'vad_eos': int(vad_eos_ms),
            'ptt': 1,
        }
    return frame


def make_tts_request(app_id, text, *, voice_name='xiaoyan', sample_rate=16000,
                     speed=50, volume=50, pitch=50):
    return {
        'common': {'app_id': app_id},
        'business': {
            'aue': 'raw',
            'auf': f'audio/L16;rate={int(sample_rate)}',
            'vcn': voice_name,
            'speed': int(speed),
            'volume': int(volume),
            'pitch': int(pitch),
            'tte': 'UTF8',
        },
        'data': {
            'status': 2,
            'text': base64.b64encode(text.encode('utf-8')).decode('ascii'),
        },
    }


def decode_tts_response(payload):
    code = int(payload.get('code', 0))
    if code != 0:
        message = payload.get('message', 'unknown iFLYTEK error')
        raise XfyunResponseError(f'iFLYTEK TTS error {code}: {message}')
    data = payload.get('data') or {}
    encoded = data.get('audio', '')
    audio = base64.b64decode(encoded) if encoded else b''
    return audio, int(data.get('status', 0)) == 2
