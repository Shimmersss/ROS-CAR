import threading
import time
import cv2
import numpy as np
import pytest
from capture_task.quality import measure,warnings
from capture_task.encode import encode_jpeg
from capture_task.frames import FrameBuffer


def test_quality_color_size(catalog):
    y,x=np.indices((480,640));gray=np.where((x//8+y//8)%2,200,60).astype(np.uint8)
    rgb=np.repeat(gray[:,:,None],3,axis=2)
    sharp=measure(rgb);blur=measure(cv2.GaussianBlur(rgb,(15,15),0))
    assert sharp['sharpness']>blur['sharpness'] and sharp['rb_ratio']==1
    assert warnings(sharp,catalog.tasks['pcb'])==[]
    assert 'width_below_minimum' in warnings(measure(rgb[::2,::2]),catalog.tasks['pcb'])
    assert 'rb_ratio_out_of_range' in warnings(measure(np.full((10,10,3),[10,10,220],np.uint8)),catalog.tasks['face'])
    decoded=cv2.imdecode(np.frombuffer(encode_jpeg(rgb),np.uint8),1)
    assert decoded.shape==(480,640,3)
    with pytest.raises(ValueError):
        encode_jpeg(rgb.astype(float))


def test_fresh_clock_and_copy():
    buffer=FrameBuffer();rgb=np.full((10,10,3),100,np.uint8)
    buffer.put(rgb,10.)
    with pytest.raises(ValueError,match='fresh_frame_timeout'):
        buffer.capture(time.monotonic(),10.,threading.Event(),.04,.5,lambda:None,lambda:10.1)
    after=time.monotonic();buffer.put(rgb,11.)
    rgb[:]=0
    frame=buffer.capture(after,10.5,threading.Event(),.1,.5,lambda:None,lambda:11.1)
    assert frame.rgb.mean()==100
    # Reversed ROS clock must not reuse a future timestamp.
    with pytest.raises(ValueError):
        buffer.capture(after,10.5,threading.Event(),.03,.5,lambda:None,lambda:10.)
