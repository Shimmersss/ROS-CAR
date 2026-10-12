import pytest
from xfyun_speech.audio_bus import AudioInbox

PCM = b'\0' * 1280


def test_only_current_capture_receives_audio():
    inbox = AudioInbox()
    inbox.push(0, PCM)
    capture = inbox.capture()
    capture.start()
    assert not inbox.frames
    inbox.push(8, PCM)
    assert capture.read(1280) == PCM
    capture.close()
    inbox.push(9, PCM)
    assert not inbox.frames


def test_gap_cancels_partial_command():
    inbox = AudioInbox()
    capture = inbox.capture()
    capture.start()
    inbox.push(1, PCM)
    capture.read(1280)
    inbox.push(3, PCM)
    with pytest.raises(RuntimeError, match='gap'):
        capture.read(1280)


def test_overrun_cancel_and_stall_fail_closed():
    inbox = AudioInbox(capacity=2)
    capture = inbox.capture()
    capture.start()
    for seq in range(3):
        inbox.push(seq, PCM)
    with pytest.raises(RuntimeError, match='overrun'):
        capture.read(1280)
    capture.close()
    capture = inbox.capture()
    capture.start()
    inbox.fail('Cancelled')
    with pytest.raises(RuntimeError, match='Cancelled'):
        capture.read(1280)
    capture.close()
    capture = inbox.capture()
    capture.start()
    with pytest.raises(TimeoutError):
        capture.read(1280)
