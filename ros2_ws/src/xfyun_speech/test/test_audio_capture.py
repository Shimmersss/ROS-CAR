import subprocess
import sys
import time
import pytest
from xfyun_speech.audio import ArecordCapture

# Actual subprocess pipes exercise select/read deadlines; these are not ALSA tests.
POPEN=subprocess.Popen

def source(monkeypatch,code,timeout=.15):
    monkeypatch.setattr('xfyun_speech.audio.subprocess.Popen',
                        lambda *a,**kw:POPEN([sys.executable,'-c',code],**kw))
    value=ArecordCapture('test',16000,read_timeout_s=timeout);value.start();return value


def test_stalled_recorder_times_out_and_releases_pipes(monkeypatch):
    capture=source(monkeypatch,'import time; time.sleep(30)')
    process=capture._process
    before=time.monotonic()
    try:
        with pytest.raises(TimeoutError,match='stalled'):capture.read(1280)
    finally:capture.close()
    assert time.monotonic()-before<2
    assert process.poll() is not None and process.stdout.closed and process.stderr.closed
    capture.close()


def test_partial_pipe_reads_are_assembled(monkeypatch):
    capture=source(monkeypatch,"import sys,time;sys.stdout.buffer.write(b'ab');sys.stdout.flush();time.sleep(.03);sys.stdout.buffer.write(b'cd');sys.stdout.flush()",timeout=1.)
    try:assert capture.read(4)==b'abcd'
    finally:capture.close()


def test_busy_process_failure_can_be_closed_and_restarted(monkeypatch):
    capture=source(monkeypatch,"import sys;sys.stderr.write('Device or resource busy');sys.exit(1)",timeout=1.)
    try:
        with pytest.raises(RuntimeError):capture.read(1280)
    finally:capture.close()
    assert capture._process is None


def test_legacy_capture_keeps_blocking_read_default(monkeypatch):
    capture=source(monkeypatch,"import sys;sys.stdout.buffer.write(b'abcd');sys.stdout.flush()",timeout=None)
    try:assert capture.read(4)==b'abcd'
    finally:capture.close()
