import socket

from xfyun_speech.asr_node import XfyunAsrNode
from xfyun_speech.recognition_gate import accept_transcript


def test_quiet_wake_needs_audio_and_substantive_text():
    assert not accept_transcript('哦哦', False, True, 2200, 2300)
    assert not accept_transcript('随便唱一首歌', False, True, 1700, 2300)
    assert accept_transcript('现在几点了', False, True, 2000, 2300)
    assert not accept_transcript('现在几点了', False, False, 2000, 2300)
    assert accept_transcript('在', True, True, 2400, 2300)


def test_dns_failure_is_retried_without_repeating_other_errors(monkeypatch):
    class FakeWebsocket:
        calls = 0

        def create_connection(self, *args, **kwargs):
            self.calls += 1
            if self.calls < 3:
                raise socket.gaierror('DNS pending')
            return object()

    fake = FakeWebsocket()
    sleeps = []
    monkeypatch.setattr('xfyun_speech.asr_node.time.sleep', sleeps.append)
    assert XfyunAsrNode._connect_with_dns_retry(lambda: 'wss://test', fake)
    assert fake.calls == 3
    assert sleeps == [1.0, 2.0]
