"""Own one model process; exiting it releases native ONNX allocations as well."""
import os
from pathlib import Path
import select
import subprocess
import time


class AsrProcess:
    def __init__(self):
        self.process = None
        self.ready = False

    @property
    def alive(self):
        process = self.process
        return process is not None and process.poll() is None

    @property
    def pid(self):
        process = self.process
        return process.pid if process is not None and process.poll() is None else 0

    def rss_kib(self):
        try:
            for line in Path(f'/proc/{self.pid}/status').read_text().splitlines():
                if line.startswith('VmRSS:'):
                    return int(line.split()[1])
        except (OSError, ValueError):
            pass
        return 0

    def start(self, command, timeout, closing):
        if self.alive:
            raise RuntimeError('ASR worker is already running')
        self.ready = False
        read_fd, write_fd = os.pipe()
        try:
            env = dict(os.environ, ROSCAR_ASR_READY_FD=str(write_fd),
                       ROSCAR_ASR_PARENT_PID=str(os.getpid()))
            self.process = subprocess.Popen(command, env=env, pass_fds=(write_fd,),
                                            start_new_session=True)
            os.close(write_fd)
            write_fd = None
            end = time.monotonic() + timeout
            while not closing.is_set():
                if not self.alive:
                    raise RuntimeError(f'ASR worker exited before model load (code {self.process.returncode})')
                remaining = end - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError('ASR model loading timed out')
                if select.select([read_fd], [], [], min(.1, remaining))[0]:
                    if os.read(read_fd, 1) != b'1' or not self.alive:
                        raise RuntimeError('ASR worker did not confirm model readiness')
                    self.ready = True
                    return
            raise RuntimeError('ASR loading cancelled or manager is shutting down')
        except BaseException:
            self.stop()
            raise
        finally:
            os.close(read_fd)
            if write_fd is not None:
                os.close(write_fd)

    def stop(self):
        self.ready = False
        process = self.process
        if process is not None:
            if process.poll() is None:
                try:
                    process.terminate()
                except ProcessLookupError:
                    pass
                try:
                    process.wait(timeout=3.)
                except subprocess.TimeoutExpired:
                    try:
                        process.kill()
                    except ProcessLookupError:
                        pass
                    process.wait(timeout=2.)
            else:
                process.wait()
            # Only report released after the process has actually exited/reaped.
            self.process = None
