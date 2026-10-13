import math
import pytest
from yolo_person_tracker.owner_lifecycle import OwnerLifecycle
from yolo_person_tracker.state import Selection
from yolo_person_tracker.owner_selection import OwnerSelection
from yolo_person_tracker.backend import Detection


def test_ten_second_loss_return_and_expiry():
    owner = OwnerLifecycle()
    owner.claim()
    assert not owner.update(False, 1.) and owner.event == 'lost'
    sequence = owner.sequence
    assert not owner.update(False, 5.) and owner.sequence == sequence
    assert not owner.update(True, 10.99) and owner.event == 'returned'
    owner.update(False, 20.)
    assert not owner.update(False, 29.99)
    assert owner.update(True, 30.) and owner.state == 'UNCLAIMED' and owner.event == 'expired'
    sequence = owner.sequence
    assert not owner.update(False, 50.) and owner.sequence == sequence
    owner.claim()
    assert owner.state == 'TRACKING' and owner.event == 'claimed'


def test_manual_release_has_no_late_expiry():
    owner = OwnerLifecycle(); owner.claim(); owner.update(False, 1.)
    owner.release()
    assert not owner.update(False, 30.) and owner.event == 'released'


@pytest.mark.parametrize('value', [0, -1, math.nan, math.inf])
def test_invalid_timeout(value):
    with pytest.raises(ValueError): OwnerLifecycle(value)


def test_single_person_cannot_replace_owner_before_release():
    selection = Selection(hold_s=0., nearest_reacquire=False)
    first, second = Detection(7, (10, 10, 60, 90), .9), Detection(8, (20, 10, 70, 90), .9)
    selection.update([first], 100, 1.)
    assert selection.lock(now=1.)[0]
    for i in range(4):
        selection.update([second], 100, 2.+i)
        assert not selection.auto_lock_single(now=2.+i)
    assert selection.target_id == (0, 7)
    selection.release()
    for i in range(3):
        selection.update([second], 100, 6.+i)
        acquired = selection.auto_lock_single(now=6.+i)
    assert acquired and selection.target_id == (0, 8)


def test_release_clears_owner_and_pending_reid_evidence():
    selection = OwnerSelection()
    selection.owner_id = 'old-owner'; selection.searching = True
    selection.frames[1] = ('camera', {'0:7'}); selection.last_evidence = 1
    selection.release()
    assert not selection.owner_id and not selection.searching and not selection.frames
    assert selection.last_evidence == 0


def test_two_second_reset_preserves_original_five_second_deadline():
    owner = OwnerLifecycle(5., 2.); owner.claim(); owner.update(False, 10.)
    assert not owner.reset_continuity_due(11.999)
    assert owner.reset_continuity_due(12.)
    assert not owner.reset_continuity_due(12.1)
    assert owner.state == 'LOST' and owner.lost_at == 10.
    assert not owner.update(False, 14.999)
    assert owner.update(True, 15.) and owner.event == 'expired'


def test_return_cancels_both_deadlines_and_next_loss_starts_fresh():
    owner = OwnerLifecycle(5., 2.); owner.claim(); owner.update(False, 10.)
    assert not owner.update(True, 11.9)
    assert not owner.reset_continuity_due(12.)
    owner.update(False, 20.); assert owner.reset_continuity_due(22.)
    assert not owner.update(True, 24.9) and owner.event == 'returned'
    owner.update(False, 30.)
    assert not owner.reset_continuity_due(31.99)
    assert owner.reset_continuity_due(32.)


@pytest.mark.parametrize('reset', [-1, math.nan, math.inf, 5., 6.])
def test_invalid_continuity_deadline(reset):
    with pytest.raises(ValueError): OwnerLifecycle(5., reset)
