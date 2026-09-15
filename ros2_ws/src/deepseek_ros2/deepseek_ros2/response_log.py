"""Append-only UTF-8 persistence for final DeepSeek answers."""

from datetime import datetime, timezone
import json
import os
from pathlib import Path


def append_response(path, answer, *, timestamp=None):
    destination = Path(path).expanduser()
    destination.parent.mkdir(parents=True, exist_ok=True)
    recorded_at = timestamp or datetime.now(timezone.utc)
    record = {
        'timestamp': recorded_at.astimezone(timezone.utc).isoformat(),
        'answer': str(answer),
    }
    descriptor = os.open(
        destination, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o600)
    with os.fdopen(descriptor, 'a', encoding='utf-8', newline='\n') as handle:
        handle.write(json.dumps(record, ensure_ascii=False) + '\n')
    return destination
