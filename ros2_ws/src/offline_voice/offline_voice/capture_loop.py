"""Single capture owner with interruptible retry; never emits synthetic audio."""
import threading


class CaptureLoop:
    def __init__(self, capture, emit, on_error, retry_s=1., max_retry_s=5.):
        if not 0 < retry_s <= max_retry_s:
            raise ValueError('invalid_capture_retry')
        self.capture, self.emit, self.on_error = capture, emit, on_error
        self.retry_s, self.max_retry_s = retry_s, max_retry_s
        self.stopped = threading.Event()
        self.lock = threading.Lock()

    def run(self):
        sequence = 0
        delay = self.retry_s
        while not self.stopped.is_set():
            try:
                # Serialize opening against shutdown so no recorder starts after stop().
                with self.lock:
                    if self.stopped.is_set():
                        return
                    self.capture.start()
                while not self.stopped.is_set():
                    pcm = self.capture.read(1280)
                    if len(pcm) != 1280:
                        raise RuntimeError('Short audio frame')
                    if self.stopped.is_set():
                        break
                    self.emit(sequence, pcm)
                    sequence += 1
                    delay = self.retry_s
            except Exception as exc:
                if not self.stopped.is_set():
                    self.on_error(exc, delay)
                    # Force consumers to invalidate an utterance across a reconnect.
                    sequence += 1
            finally:
                # Always release the previous process before opening another owner.
                with self.lock:
                    self.capture.close()
            if self.stopped.wait(delay):
                break
            delay = min(self.max_retry_s, delay*2)

    def stop(self):
        self.stopped.set()
        with self.lock:
            self.capture.close()  # Interrupt a read; shutdown never waits for retry backoff.
