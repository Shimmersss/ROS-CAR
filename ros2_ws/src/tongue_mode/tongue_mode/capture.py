"""Receive-time window selection; never subtract ROS timestamps from monotonic time."""
from dataclasses import dataclass
from io import BytesIO
import threading
import time
from PIL import Image
from .cloud import Cancelled
from .results import AnalysisError


@dataclass
class Frame:
    rgb: object
    sequence: int
    received: float
    ros_stamp: float


class FrameBuffer:
    def __init__(self):
        self._condition = threading.Condition()
        self._frame = None
        self._seq = 0

    def put(self, rgb, ros_stamp=0.):
        with self._condition:
            self._seq += 1
            self._frame = Frame(rgb.copy(), self._seq, time.monotonic(), ros_stamp)
            self._condition.notify_all()

    def capture(self, after, cancel, timeout=2., max_age=.5, tick=lambda: None, ros_after=None):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            if cancel.is_set():
                raise Cancelled()
            tick()
            with self._condition:
                frame = self._frame
                if (frame and frame.received > after and time.monotonic()-frame.received <= max_age
                        and (ros_after is None or frame.ros_stamp > ros_after)):
                    return frame
                self._condition.wait(.05)
        raise AnalysisError('No fresh RGB frame after prompt')


def encode_jpeg(frame, quality=95):
    rgb = frame.rgb
    if len(rgb.shape) != 3 or rgb.shape[2] != 3 or str(rgb.dtype) != 'uint8' or rgb.shape[0]*rgb.shape[1] > 40000000:
        raise AnalysisError('Invalid RGB image')
    buf = BytesIO()
    Image.fromarray(rgb, 'RGB').save(buf, 'JPEG', quality=quality, subsampling=0)
    if buf.tell() > 12*1024*1024:
        raise AnalysisError('Encoded image exceeds upload limit')
    return buf.getvalue()
