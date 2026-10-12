import pytest
from offline_voice.frontend import qwen_hotword_options


def test_disabled_hotwords_do_not_change_asr():
    assert qwen_hotword_options(lambda: None, [], False) == {}


def test_native_support_is_required():
    with pytest.raises(ValueError, match='does not support'):
        qwen_hotword_options(lambda: None, ['前进'], True)
    assert qwen_hotword_options(lambda hotwords='': None, ['前进', '后退'], True) == {'hotwords': '前进,后退'}


@pytest.mark.parametrize('words', [[], [''], ['前进,后退'], ['前进'] * 21])
def test_only_small_explicit_hotword_set_is_allowed(words):
    with pytest.raises(ValueError):
        qwen_hotword_options(lambda hotwords='': None, words, True)
