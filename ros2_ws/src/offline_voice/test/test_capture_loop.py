import threading
import time
from offline_voice.capture_loop import CaptureLoop


class Capture:
    def __init__(self, failures=0):
        self.failures=failures;self.starts=0;self.active=False;self.reads=0
    def start(self):
        assert not self.active, 'previous recorder must be closed before retry'
        self.starts+=1
        if self.starts<=self.failures:raise RuntimeError('device busy')
        self.active=True
    def read(self,size):
        assert self.active
        self.reads+=1
        return b'\x12'*size
    def close(self):self.active=False


def test_busy_at_boot_recovers_without_duplicate_owner():
    source=Capture(failures=2);frames=[];errors=[]
    def emit(sequence,pcm):
        frames.append((sequence,pcm));loop.stop()
    loop=CaptureLoop(source,emit,lambda exc,delay:errors.append(delay),retry_s=.001,max_retry_s=.002)
    loop.run()
    assert source.starts==3 and not source.active
    assert errors==[.001,.002] and frames==[(2,b'\x12'*1280)]


def test_reconnect_invalidates_sequence_and_only_emits_complete_frames():
    class Disconnect(Capture):
        def read(self,size):
            if self.starts==1 and self.reads==2:raise TimeoutError('audio stalled')
            return super().read(size)
    source=Disconnect();frames=[]
    def emit(seq,pcm):
        frames.append(seq)
        if len(frames)==3:loop.stop()
    loop=CaptureLoop(source,emit,lambda *_:None,retry_s=.001,max_retry_s=.002)
    loop.run()
    assert frames==[0,1,3] and source.starts==2 and not source.active


def test_short_frame_is_not_published():
    class Short(Capture):
        def read(self,size):return b'\x00'
    frames=[];source=Short()
    loop=CaptureLoop(source,lambda *v:frames.append(v),lambda *_:loop.stop())
    loop.run()
    assert not frames and not source.active


def test_stop_interrupts_backoff_and_never_reopens_after_shutdown():
    source=Capture(failures=100);failed=threading.Event()
    loop=CaptureLoop(source,lambda *_:None,lambda *_:failed.set(),retry_s=5,max_retry_s=5)
    worker=threading.Thread(target=loop.run);worker.start()
    assert failed.wait(1)
    before=time.monotonic();loop.stop();worker.join(1)
    assert not worker.is_alive() and time.monotonic()-before<1 and source.starts==1
    loop.run();assert source.starts==1
