"""Real OS processes and memory; no ROS or model weights required."""
import os
from pathlib import Path
import signal
import sys
import threading
import time

import pytest
from offline_voice.asr_process import AsrProcess

pytestmark = pytest.mark.skipif(not sys.platform.startswith('linux'),
    reason='ASR process lifetime and /proc RSS checks require Linux')


def command(body):
    return [sys.executable, '-c', 'import os,time,signal\n' + body]


def test_model_memory_and_idempotent_stop():
    worker = AsrProcess()
    try:
        worker.start(command("memory=bytearray(48*1024*1024)\nos.write(int(os.environ['ROSCAR_ASR_READY_FD']),b'1')\ntime.sleep(60)"),
                     3., threading.Event())
        pid = worker.pid
        assert worker.ready and worker.rss_kib() > 48*1024
        worker.stop()
        assert worker.pid == 0 and worker.rss_kib() == 0 and not worker.ready
        assert not Path(f'/proc/{pid}').exists()
        worker.stop()
    finally:
        worker.stop()


@pytest.mark.parametrize('body', ["raise SystemExit(7)", "os.write(int(os.environ['ROSCAR_ASR_READY_FD']),b'x')\ntime.sleep(60)"])
def test_failed_load_reaps_process(body):
    worker = AsrProcess()
    with pytest.raises(RuntimeError):
        worker.start(command(body), 3., threading.Event())
    assert not worker.ready and worker.process is None


def test_timeout_and_shutdown_do_not_leave_model_process():
    worker = AsrProcess()
    with pytest.raises(TimeoutError):
        worker.start(command('time.sleep(60)'), .15, threading.Event())
    assert worker.process is None
    closing = threading.Event()
    closing.set()
    with pytest.raises(RuntimeError, match='shutting down'):
        worker.start(command('time.sleep(60)'), 3., closing)
    assert worker.process is None


def test_missing_executable_closes_pipe_and_recovers():
    worker = AsrProcess()
    before = len(list(Path('/proc/self/fd').iterdir()))
    for _ in range(3):
        with pytest.raises(FileNotFoundError):
            worker.start(['/nonexistent/roscar-asr'], 1., threading.Event())
    assert len(list(Path('/proc/self/fd').iterdir())) == before


def test_native_work_ignoring_term_is_killed_and_reaped():
    worker = AsrProcess()
    try:
        worker.start(command("signal.signal(signal.SIGTERM,signal.SIG_IGN)\nos.write(int(os.environ['ROSCAR_ASR_READY_FD']),b'1')\ntime.sleep(60)"),
                     3., threading.Event())
        process = worker.process
        worker.stop()
        assert process.returncode == -signal.SIGKILL and worker.process is None
    finally:
        worker.stop()


def test_repeated_reload_gets_fresh_processes():
    worker = AsrProcess()
    pids = []
    try:
        for _ in range(3):
            worker.start(command("os.write(int(os.environ['ROSCAR_ASR_READY_FD']),b'1')\ntime.sleep(60)"),
                         3., threading.Event())
            pids.append(worker.pid)
            worker.stop()
        assert len(set(pids)) == 3
    finally:
        worker.stop()


def test_manager_kill_cannot_leave_orphan_model_memory(tmp_path):
    import subprocess
    pid_file = tmp_path/'worker.pid'
    child = ("from offline_voice.asr_worker import protect_parent_lifetime\n"
             "protect_parent_lifetime()\n"
             "memory=bytearray(48*1024*1024)\n"
             "os.write(int(os.environ['ROSCAR_ASR_READY_FD']),b'1')\ntime.sleep(60)")
    outer = ("import os,sys,signal,threading\nfrom pathlib import Path\n"
             "from offline_voice.asr_process import AsrProcess\n"
             "worker=AsrProcess()\n"
             f"worker.start({command(child)!r},3.,threading.Event())\n"
             f"Path({str(pid_file)!r}).write_text(str(worker.pid))\n"
             "os.kill(os.getpid(),signal.SIGKILL)\n")
    result = subprocess.run([sys.executable,'-c',outer],timeout=8.)
    assert result.returncode == -signal.SIGKILL
    pid = int(pid_file.read_text())
    end = time.monotonic()+3.
    while time.monotonic()<end:
        try:
            status=Path(f'/proc/{pid}/status').read_text()
        except FileNotFoundError:
            return
        if 'State:\tZ' in status:  # A zombie has no model address space.
            assert 'VmRSS:' not in status
            return
        time.sleep(.02)
    os.kill(pid,signal.SIGKILL)
    pytest.fail('ASR worker survived its manager')
