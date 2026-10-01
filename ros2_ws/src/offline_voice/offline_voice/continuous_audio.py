"""Frame alignment for a continuously running voice activity detector."""

import numpy as np


class VadSegmenter:
    """Feed exact VAD windows and copy completed speech before popping it."""

    def __init__(self, vad, window_size):
        self.vad = vad
        self.window_size = int(window_size)
        if self.window_size <= 0:
            raise ValueError('VAD window_size must be positive')
        self._pending = np.empty(0, dtype=np.float32)

    def accept(self, samples):
        samples = np.asarray(samples, dtype=np.float32)
        if samples.size:
            self._pending = np.concatenate((self._pending, samples))
        offset = 0
        while self._pending.size - offset >= self.window_size:
            self.vad.accept_waveform(self._pending[offset:offset + self.window_size])
            offset += self.window_size
        if offset:
            self._pending = self._pending[offset:].copy()
        segments = []
        while not self.vad.empty():
            segments.append(np.asarray(self.vad.front.samples, dtype=np.float32).copy())
            self.vad.pop()
        return segments

    def reset(self):
        self.vad.reset()
        self._pending = np.empty(0, dtype=np.float32)
