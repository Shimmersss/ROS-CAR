import math
import pytest
from yolo_person_tracker.stationary import StationaryGate


def feed(g,t,v=0.,w=0.):
    return g.observe(100.+t,v,w,t,100.+t)


def test_stationary_needs_continuous_fresh_half_second():
    g=StationaryGate()
    for i in range(5):
        feed(g,i*.1);assert not g.ready(i*.1,100.+i*.1,100.+i*.1)
    feed(g,.5);assert g.ready(.5,100.5,100.5)
    assert not g.ready(.9,100.9,100.9)
    feed(g,.9);assert not g.ready(.9,100.9,100.9)


@pytest.mark.parametrize('v,w',[(.02,0.),(-.02,0.),(0.,.05),(math.nan,0.),(0.,math.inf)])
def test_motion_or_invalid_resets_stable_interval(v,w):
    g=StationaryGate()
    for i in range(7):feed(g,i*.1)
    assert not feed(g,.7,v,w)
    feed(g,.8);assert not g.ready(.8,100.8,100.8)


def test_duplicate_future_and_pre_stop_frame_rejected():
    g=StationaryGate()
    for i in range(7):feed(g,i*.1)
    assert not g.ready(.6,100.6,99.)
    assert not g.observe(100.6,0.,0.,.7,100.7)
    assert not g.observe(101.,0.,0.,.8,100.8)
