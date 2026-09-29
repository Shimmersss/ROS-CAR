import numpy as np

from offline_voice.continuous_audio import VadSegmenter


class FakeVad:
    def __init__(self):
        self.windows = []
        self.completed = []

    def accept_waveform(self, samples):
        self.windows.append(samples.copy())
        if len(self.windows) == 3:
            self.completed.append(np.concatenate(self.windows))

    def empty(self):
        return not self.completed

    @property
    def front(self):
        return type('Segment', (), {'samples': self.completed[0]})()

    def pop(self):
        self.completed.pop(0)

    def reset(self):
        self.windows.clear()
        self.completed.clear()


def test_vad_windows_span_capture_reads_without_losing_samples():
    vad = FakeVad()
    segmenter = VadSegmenter(vad, window_size=4)
    assert segmenter.accept(np.arange(3)) == []
    assert segmenter.accept(np.arange(3, 7)) == []
    result = segmenter.accept(np.arange(7, 15))
    assert len(result) == 1
    assert result[0].tolist() == list(range(12))
    assert np.concatenate(vad.windows).tolist() == list(range(12))


def test_reset_discards_partial_utterance():
    vad = FakeVad()
    segmenter = VadSegmenter(vad, window_size=4)
    segmenter.accept(np.arange(6))
    segmenter.reset()
    assert segmenter.accept(np.arange(10, 22))[0].tolist() == list(range(10, 22))
