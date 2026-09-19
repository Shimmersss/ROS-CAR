"""Short acknowledgement replies, answered locally without calling the API.

After the hardware wake word the owner often says only a single syllable such as
``在``. Sending that to DeepSeek wastes a round trip and produces an arbitrary
answer, while the natural response is a fixed one. This module holds the pure
matching logic so it is unit-testable without ROS or network access.
"""

from dataclasses import dataclass
import re

# ASR may return punctuation (``在。``), full-width marks or spacing. Strip
# everything that is not a CJK character or a Latin letter/digit, so matching
# only ever compares the words themselves.
_NOISE = re.compile(r'[^\u4e00-\u9fffA-Za-z0-9]')


@dataclass(frozen=True)
class AcknowledgementConfig:
    """Trigger phrases and the fixed reply used for them."""

    phrases: tuple = ('在', '我在', '在这儿', '在这里')
    reply: str = '我在'
    max_phrase_chars: int = 8

    def __post_init__(self):
        if not isinstance(self.phrases, (tuple, list, set, frozenset)):
            raise TypeError('acknowledgement phrases must be a sequence')
        if not isinstance(self.reply, str) or not self.reply.strip():
            raise ValueError('acknowledgement reply must not be empty')
        if self.max_phrase_chars < 1:
            raise ValueError('max_phrase_chars must be positive')


def normalize(text):
    """Drop punctuation and whitespace so ASR formatting cannot hide a match."""
    return _NOISE.sub('', text or '')


def normalize_phrases(phrases):
    """Normalise the configured phrases, skipping empty ones and duplicates."""
    seen = {}
    for phrase in phrases or ():
        key = normalize(phrase)
        if key:
            seen.setdefault(key, phrase)
    return tuple(seen)


def match_wake_trigger(text, trigger):
    """True when `text` is the wake event that should be greeted.

    The vendor publishes this string on every hardware wake, so it must match
    exactly; normalisation only guards against spacing the driver may add.
    """
    expected = normalize(trigger)
    return bool(expected) and normalize(text) == expected


def match_acknowledgement(text, config=AcknowledgementConfig()):
    """Return the fixed reply when `text` is a bare acknowledgement.

    Matching compares the whole normalised utterance, never a substring: ``在吗``
    or ``在哪里`` are real questions and must still reach the model. Only short
    utterances are considered, so a long sentence that merely contains ``在`` is
    never swallowed either.
    """
    normalized = normalize(text)
    if not normalized or len(normalized) > config.max_phrase_chars:
        return None
    if normalized in normalize_phrases(config.phrases):
        return config.reply
    return None
