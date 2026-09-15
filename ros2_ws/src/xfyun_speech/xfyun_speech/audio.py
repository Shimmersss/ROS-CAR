"""Small ALSA command wrappers with no shell interpolation."""

import math
import subprocess


def pcm_rms_s16le(data):
    """Calculate RMS for little-endian signed 16-bit mono PCM."""
    usable = len(data) - (len(data) % 2)
    if usable == 0:
        return 0.0
    samples = memoryview(data[:usable]).cast('h')
    return math.sqrt(sum(sample * sample for sample in samples) / len(samples))


class ArecordCapture:
    def __init__(self, device, sample_rate, channels=1):
        self._command = [
            'arecord', '-q', '-D', device, '-t', 'raw', '-f', 'S16_LE',
            '-r', str(sample_rate), '-c', str(channels),
        ]
        self._process = None

    def start(self):
        self._process = subprocess.Popen(
            self._command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

    def read(self, byte_count):
        if self._process is None or self._process.stdout is None:
            raise RuntimeError('arecord is not running')
        data = self._process.stdout.read(byte_count)
        if data:
            return data
        detail = ''
        if self._process.stderr is not None:
            detail = self._process.stderr.read().decode('utf-8', errors='replace').strip()
        raise RuntimeError(detail or 'arecord stopped before producing audio')

    def close(self):
        if self._process is None:
            return
        if self._process.poll() is None:
            self._process.terminate()
            try:
                self._process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait(timeout=2)
        self._process = None


class AplaySink:
    def __init__(self, device, sample_rate, channels=1):
        self._command = [
            'aplay', '-q', '-D', device, '-t', 'raw', '-f', 'S16_LE',
            '-r', str(sample_rate), '-c', str(channels),
        ]
        self._process = None

    def start(self):
        self._process = subprocess.Popen(
            self._command,
            stdin=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

    def write(self, data):
        if not data:
            return
        if self._process is None or self._process.stdin is None:
            raise RuntimeError('aplay is not running')
        self._process.stdin.write(data)
        self._process.stdin.flush()

    def close(self):
        if self._process is None:
            return
        if self._process.stdin is not None and not self._process.stdin.closed:
            self._process.stdin.close()
        try:
            return_code = self._process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self._process.kill()
            return_code = self._process.wait(timeout=2)
        if return_code != 0:
            detail = ''
            if self._process.stderr is not None:
                detail = self._process.stderr.read().decode('utf-8', errors='replace').strip()
            raise RuntimeError(detail or f'aplay exited with status {return_code}')
        self._process = None
