"""Local evidence required before forwarding a cloud transcript."""

from .protocol import normalize_phrase


def accept_transcript(text, speech_started, trusted_wake, peak_rms,
                      energy_threshold, near_threshold_ratio=0.8,
                      min_quiet_chars=4):
    """Allow quiet speech after a hardware wake, but reject noise-only guesses."""
    if not text or not text.strip():
        return False
    if speech_started:
        return True
    return (
        trusted_wake
        and peak_rms >= energy_threshold * near_threshold_ratio
        and len(normalize_phrase(text)) >= min_quiet_chars
    )
