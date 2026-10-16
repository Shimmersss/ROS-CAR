import math
import pytest
from astra_body_adapter.fall_pause import FallPause


def feed(p, stage, t, valid=True):
    p.observe(valid, stage, t+100., t, t+100.)
    return p.update(t, t+100.)


def test_fresh_fall_and_continuous_none_required():
    p=FallPause()
    assert feed(p, 1, 1.)
    assert feed(p, 2, 1.1)
    for i in range(15): assert feed(p, 0, 2.+i*.2)
    assert not feed(p, 0, 5.)


def test_invalid_stale_duplicate_or_clock_reversal_never_clear():
    for valid, stamp in [(False,102.), (True,100.), (True,101.), (True,200.)]:
        p=FallPause(); feed(p,2,1.)
        p.observe(valid,0,stamp,2.,102.)
        assert p.update(20.,120.) and not p.fresh(20.,120.)
        assert not p.resume(20.,120.)


def test_none_gap_and_unusable_target_reset_or_delay_resume():
    p=FallPause();feed(p,1,1.);feed(p,0,2.)
    assert p.update(3.,103.)
    for i in range(15): assert feed(p,0,4.+i*.2)
    p.observe(True,0,107.,7.,107.)
    assert p.update(7.,107.,can_resume=False)
    assert not p.update(7.,107.,can_resume=True)


def test_manual_resume_requires_clear_fresh_evidence():
    p=FallPause();feed(p,2,1.)
    assert not p.resume(1.,101.)
    feed(p,0,1.1)
    assert p.resume(1.1,101.1) and not p.paused
    assert feed(p,1,1.2)


@pytest.mark.parametrize('v',[0.,-1.,math.inf,math.nan])
def test_bad_timing(v):
    with pytest.raises(ValueError): FallPause(resume_s=v)
