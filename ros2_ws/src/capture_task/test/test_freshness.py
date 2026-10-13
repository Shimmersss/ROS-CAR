import threading
import time
import numpy as np
import pytest
from capture_task.frames import FrameBuffer, Frame
from capture_task.state import fresh, mode_active, MODE_MAX_AGE, EXECUTOR_MAX_AGE


@pytest.mark.parametrize('offset,accepted',[(.03,True),(.08,False),(-.4,True),(-.6,False)])
def test_ros_stamp_tolerance(offset,accepted):
    buffer=FrameBuffer();after=time.monotonic()-.1
    buffer.put(np.zeros((2,2,3),np.uint8),100.+offset)
    args=(after,99.,threading.Event(),.02,.5,lambda:None,lambda:100.)
    if accepted:
        assert buffer.capture(*args).ros_stamp==100.+offset
    else:
        with pytest.raises(ValueError,match='fresh_frame_timeout'):buffer.capture(*args)


def test_window_and_monotonic_age_remain_strict():
    buffer=FrameBuffer();after=time.monotonic()-.1
    buffer.put(np.zeros((2,2,3),np.uint8),100.)
    with pytest.raises(ValueError,match='fresh_frame_timeout'):
        buffer.capture(after,100.01,threading.Event(),.02,.5,lambda:None,lambda:100.)
    buffer.latest=Frame(buffer.latest.rgb,1,time.monotonic()+1.,100.)
    with pytest.raises(ValueError,match='fresh_frame_timeout'):
        buffer.capture(after,99.,threading.Event(),.02,.5,lambda:None,lambda:100.)


@pytest.mark.parametrize('limit',[MODE_MAX_AGE,EXECUTOR_MAX_AGE])
def test_state_age_boundaries(limit):
    assert fresh(10.,limit,10.)
    assert fresh(10.,limit,10.+limit)
    assert not fresh(10.,limit,10.+limit+.001)
    assert not fresh(10.,limit,9.99)
    assert not fresh(0.,limit,0.)


def test_mode_never_received_or_expired_is_inactive():
    assert not mode_active({'active':True},0.,10.)
    assert mode_active({'active':True},10.,16.5)
    assert not mode_active({'active':True},10.,16.501)
    assert not mode_active({},10.,10.)
