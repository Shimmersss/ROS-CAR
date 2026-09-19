"""Guards for the early-finalisation gate used by the wake acknowledgement.

`xfyun_speech.early_finalize` carries no ROS import, so this runs anywhere. The
node-level wiring and the real audio path are exercised on the vehicle; a
syntax error in `asr_node` itself must be caught by compiling it (see the local
check in the project workflow), because importing it needs rclpy.
"""

import pytest

from xfyun_speech.early_finalize import early_finalize_ready

PHRASES = ['在', '我在', '在这儿', '在这里']
NOW = 100.0


def gate(text, elapsed, minimum=0.25, deadline=NOW + 5.0, phrases=PHRASES):
    return early_finalize_ready(text, elapsed, minimum, deadline, phrases, NOW)


@pytest.mark.parametrize('text', ['在', '在。', ' 在 ', '我在', '在这儿'])
def test_listed_phrase_after_minimum_elapsed_finalises(text):
    assert gate(text, 0.4)


def test_question_never_finalises_early():
    # Truncating a real question would lose the actual request.
    for text in ('在吗', '在哪里', '现在几点了', '在不在'):
        assert not gate(text, 1.0)


def test_too_early_is_rejected():
    # Partial results are unstable at the very start of an utterance.
    assert not gate('在', 0.1)


def test_after_the_window_is_rejected():
    assert not gate('在', 0.4, deadline=NOW - 0.1)


def test_empty_phrase_list_disables_the_gate():
    assert not gate('在', 1.0, phrases=[])
    assert not gate('在', 1.0, phrases=None)


def test_empty_or_missing_text_is_rejected():
    assert not gate('', 1.0)
    assert not gate(None, 1.0)
