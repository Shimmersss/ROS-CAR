"""Job identity and completion shared by local and online TTS workers."""
from dataclasses import dataclass, field
import threading


@dataclass
class SpeechJob:
    text: str
    utterance_id: str
    cancel: threading.Event = field(default_factory=threading.Event)
    complete: threading.Event = field(default_factory=threading.Event)
    status: str = 'queued'
    detail: str = ''
