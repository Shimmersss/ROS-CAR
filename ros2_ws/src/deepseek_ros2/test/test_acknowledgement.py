import pytest

from deepseek_ros2.acknowledgement import (
    AcknowledgementConfig,
    match_acknowledgement,
    match_wake_trigger,
    normalize,
    normalize_phrases,
)


@pytest.mark.parametrize('text', [
    '在', '我在', '在这儿', '在这里',
    '在。', '我在！', ' 在 ', '在，', '“我在”', '在．',
])
def test_bare_acknowledgements_are_matched(text):
    assert match_acknowledgement(text) == '我在'


@pytest.mark.parametrize('text', [
    '在吗', '在哪里', '你在干什么', '我现在在车上', '不在',
    '现在几点了', '在不在', '我在想一个问题',
])
def test_questions_and_long_sentences_are_not_swallowed(text):
    assert match_acknowledgement(text) is None


@pytest.mark.parametrize('text', ['', '   ', '。。。', None])
def test_empty_or_punctuation_only_text_is_ignored(text):
    assert match_acknowledgement(text) is None


def test_matching_is_whole_utterance_not_substring():
    # A long utterance containing "在" must reach the model untouched.
    assert match_acknowledgement('请问洗手间在哪里') is None


def test_over_long_utterance_is_rejected_before_comparison():
    config = AcknowledgementConfig(phrases=('在',), max_phrase_chars=1)
    assert match_acknowledgement('在', config) == '我在'
    assert match_acknowledgement('我在', config) is None


def test_custom_phrases_and_reply_are_honoured():
    config = AcknowledgementConfig(phrases=('到', '收到'), reply='到！')
    assert match_acknowledgement('到', config) == '到！'
    assert match_acknowledgement('收到。', config) == '到！'
    assert match_acknowledgement('在', config) is None


def test_punctuation_and_spacing_are_removed():
    assert normalize(' 我在，。！ ') == '我在'


def test_phrase_normalisation_drops_empty_and_duplicate_entries():
    assert normalize_phrases(['在', ' 在 ', '', '我在']) == ('在', '我在')


def test_empty_phrases_never_match():
    config = AcknowledgementConfig(phrases=())
    assert match_acknowledgement('在', config) is None


def test_reply_must_not_be_empty():
    with pytest.raises(ValueError):
        AcknowledgementConfig(reply='   ')


def test_phrase_container_is_type_checked():
    with pytest.raises(TypeError):
        AcknowledgementConfig(phrases='在')


def test_max_phrase_chars_must_be_positive():
    with pytest.raises(ValueError):
        AcknowledgementConfig(max_phrase_chars=0)


@pytest.mark.parametrize('text', ['小车唤醒', '小车唤醒 ', ' 小车唤醒'])
def test_wake_trigger_matches_the_driver_event(text):
    assert match_wake_trigger(text, '小车唤醒')


@pytest.mark.parametrize('text', [
    '小车', '唤醒', '小微小微', '小车唤醒了吗', '', None, '在',
])
def test_wake_trigger_rejects_anything_else(text):
    # A greeting must never fire on a fragment or on the user's question.
    assert not match_wake_trigger(text, '小车唤醒')


def test_wake_trigger_without_a_configured_word_never_fires():
    assert not match_wake_trigger('小车唤醒', '')
    assert not match_wake_trigger('小车唤醒', None)
