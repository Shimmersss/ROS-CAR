"""Bounded PCM handoff; a dropped/out-of-order frame cancels the utterance."""
from collections import deque
import threading
import time


class AudioInbox:
    def __init__(self, capacity=40):
        self.frames = deque(maxlen=capacity)
        self.condition = threading.Condition()
        self.listening = False
        self.error = None

    def push(self, sequence, pcm):
        if not isinstance(sequence, int) or sequence < 0 or len(pcm) != 1280:
            self.fail('Invalid PCM frame')
            return
        with self.condition:
            if self.listening:
                if len(self.frames) == self.frames.maxlen:
                    self.error = 'Audio queue overrun'
                self.frames.append((sequence, bytes(pcm)))
                self.condition.notify_all()

    def fail(self, error):
        with self.condition:
            self.error = str(error)
            self.condition.notify_all()

    def capture(self):
        return InboxCapture(self)


class InboxCapture:
    def __init__(self, inbox):
        self.inbox = inbox
        self.sequence = None

    def start(self):
        with self.inbox.condition:
            self.inbox.frames.clear()
            self.inbox.error = None
            self.inbox.listening = True

    def read(self, size):
        if size != 1280:
            raise ValueError('Shared audio requires 16 kHz/mono/40 ms frames')
        deadline = time.monotonic() + .3
        with self.inbox.condition:
            while not self.inbox.frames and not self.inbox.error:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError('Shared audio stalled')
                self.inbox.condition.wait(remaining)
            if self.inbox.error:
                raise RuntimeError(self.inbox.error)
            sequence, pcm = self.inbox.frames.popleft()
        if self.sequence is not None and sequence != self.sequence + 1:
            raise RuntimeError('Audio sequence gap; discard partial utterance')
        self.sequence = sequence
        return pcm

    def close(self):
        with self.inbox.condition:
            self.inbox.listening = False
            self.inbox.frames.clear()
