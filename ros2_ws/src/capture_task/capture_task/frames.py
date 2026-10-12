"""Fresh RGB buffer, adapted from tongue_mode/capture.py; clocks stay separate."""
from dataclasses import dataclass
import threading
import time
import numpy as np
from .cloud_client import Cancelled


@dataclass(frozen=True)
class Frame:
    rgb: np.ndarray
    sequence: int
    received: float
    ros_stamp: float


class FrameBuffer:
    def __init__(self):
        self.condition = threading.Condition()
        self.latest = None
        self.sequence = 0

    def put(self, rgb, ros_stamp):
        if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3 or not 0 < rgb.shape[0]*rgb.shape[1] <= 40000000:
            raise ValueError('invalid_rgb')
        with self.condition:
            self.sequence += 1
            self.latest = Frame(rgb.copy(), self.sequence, time.monotonic(), ros_stamp)
            self.condition.notify_all()

    def capture(self, after, ros_after, cancel, timeout, max_age, check, ros_now):
        end = time.monotonic()+timeout
        while True:
            if cancel.is_set():
                raise Cancelled()
            check()
            now = time.monotonic()
            with self.condition:
                frame = self.latest
                if frame and frame.received > after and frame.ros_stamp > ros_after and 0 <= now-frame.received <= max_age and 0 <= ros_now()-frame.ros_stamp <= max_age:
                    return Frame(frame.rgb.copy(),frame.sequence,frame.received,frame.ros_stamp)
                if now >= end:
                    raise ValueError('fresh_frame_timeout')
                self.condition.wait(min(.025, end-now))
