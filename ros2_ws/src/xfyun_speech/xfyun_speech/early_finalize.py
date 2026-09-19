"""Decide when a short wake utterance can be finalised without waiting.

Kept free of ROS so the rule is unit-testable on any machine. `make_iat_frame`
still produces the end-of-audio frame the server needs; this module only decides
*when* to send it, which is what removes the silence-window delay for a bare
"在".
"""

from .protocol import phrase_matches


def early_finalize_ready(text, elapsed, minimum_elapsed, deadline, phrases, now):
    """True when the partial result is already a complete short utterance.

    Never invents text: the server still returns the final result after the
    end-of-audio frame. Guards, in order:

    - no configured phrases (or no text yet) means the gate is disabled;
    - past `deadline` the caller no longer expects a wake utterance, so normal
      silence handling applies;
    - below `minimum_elapsed` the partial result is still unstable at the start
      of an utterance;
    - and the whole utterance must equal a listed phrase, so questions such as
      在吗/在哪里 are never cut short.
    """
    if not phrases or not text:
        return False
    if now > deadline:
        return False
    if elapsed < minimum_elapsed:
        return False
    return phrase_matches(text, phrases)
