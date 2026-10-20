"""Small ALSA command wrappers with no shell interpolation."""

import math
import os
import select
import time
import subprocess


def pcm_rms_s16le(data):
    """Calculate RMS for little-endian signed 16-bit mono PCM."""
    usable = len(data) - (len(data) % 2)
    if usable == 0:
        return 0.0
    samples = memoryview(data[:usable]).cast('h')
    return math.sqrt(sum(sample * sample for sample in samples) / len(samples))


class ArecordCapture:
    def __init__(self, device, sample_rate, channels=1, read_timeout_s=None):
        if read_timeout_s is not None and (not math.isfinite(read_timeout_s) or read_timeout_s <= 0):
            raise ValueError('invalid_capture_timeout')
        self.read_timeout_s = read_timeout_s
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
        process = self._process
        if process is None or process.stdout is None:
            raise RuntimeError('arecord is not running')
        if self.read_timeout_s is None:
            data = process.stdout.read(byte_count)
        else:
            if byte_count <= 0:
                raise ValueError('invalid_audio_read_size')
            deadline = time.monotonic()+self.read_timeout_s
            chunks = bytearray()
            while len(chunks) < byte_count:
                remaining = deadline-time.monotonic()
                if remaining <= 0 or not select.select([process.stdout], [], [], remaining)[0]:
                    raise TimeoutError('arecord audio stalled')
                chunk = os.read(process.stdout.fileno(), byte_count-len(chunks))
                if not chunk:
                    break
                chunks.extend(chunk)
            data = bytes(chunks)
        if data:
            return data
        detail = ''
        if process.poll() is not None and process.stderr is not None:
            detail = process.stderr.read().decode('utf-8', errors='replace').strip()
        raise RuntimeError(detail or 'arecord stopped before producing audio')

    def close(self):
        process = self._process
        if process is None:
            return
        try:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=2)
        finally:
            for stream in (process.stdout, process.stderr):
                if stream is not None:
                    stream.close()
            self._process = None


class AplaySink:
    def __init__(self, device, sample_rate, channels=1):
        self._command = [
            'aplay', '-q', '-D', device, '-t', 'raw', '-f', 'S16_LE',
            '-r', str(sample_rate), '-c', str(channels),
        ]
        self._process = None
        self._aborted = False

    def start(self):
        if self._aborted:
            raise RuntimeError('Playback cancelled')
        self._process = subprocess.Popen(
            self._command,
            stdin=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        if self._aborted:
            self.abort()
            self.close()
            raise RuntimeError('Playback cancelled')

    def write(self, data):
        if self._aborted:
            raise RuntimeError('Playback cancelled')
        if not data:
            return
        if self._process is None or self._process.stdin is None:
            raise RuntimeError('aplay is not running')
        self._process.stdin.write(data)
        self._process.stdin.flush()

    def abort(self):
        self._aborted = True
        process = self._process
        if process is not None and process.poll() is None:
            try:
                process.terminate()
            except ProcessLookupError:
                pass

    def close(self):
        if self._process is None:
            return
        if self._process.stdin is not None and not self._process.stdin.closed:
            try:
                self._process.stdin.close()
            except BrokenPipeError:
                if not self._aborted:
                    raise
        try:
            return_code = self._process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self._process.kill()
            return_code = self._process.wait(timeout=2)
        if return_code != 0 and not self._aborted:
            detail = ''
            if self._process.stderr is not None:
                detail = self._process.stderr.read().decode('utf-8', errors='replace').strip()
            raise RuntimeError(detail or f'aplay exited with status {return_code}')
        self._process = None
