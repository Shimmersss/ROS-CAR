"""Sparse repair must match independent per-hole filling, without mutating camera depth."""
from unittest.mock import patch
import numpy as np
import pytest
from yolo_person_tracker.depth import _measure_region

@pytest.mark.parametrize('dtype',[np.float32,np.float64])
def test_sparse_holes_match_per_pixel_reference_and_do_not_recompute_valid_pixels(dtype):
    rng=np.random.default_rng(32)
    depth=rng.uniform(1.8,2.2,(60,80)).astype(dtype)
    depth[rng.random(depth.shape)<.1]=0.
    depth[20:24,20:24]=np.nan
    depth[0,0]=np.inf
    original=depth.copy()
    valid=np.isfinite(depth)&(depth>=.2)&(depth<=8.)
    expected=depth.copy();filled=0
    # Independent oracle: inspect each missing pixel using only the original
    # neighbours, never cascading filled pixels into another hole.
    for y,x in zip(*np.nonzero(~valid)):
        neighbours=depth[max(0,y-1):y+2,max(0,x-1):x+2]
        values=neighbours[np.isfinite(neighbours)&(neighbours>=.2)&(neighbours<=8.)]
        if len(values)>=5:
            expected[y,x]=np.float32(np.median(values));filled+=1
    intr=(100.,100.,40.,30.);bounds=(0,0,80,60)
    sort=np.sort
    with patch('yolo_person_tracker.depth.np.sort',wraps=sort) as repair:
        got=_measure_region(depth,bounds,intr,.15)
    sparse=[call for call in repair.call_args_list if call.args[0].ndim==2]
    assert len(sparse)==1
    assert sparse[0].args[0].shape==(9,filled)
    reference=_measure_region(expected,bounds,intr,.15,repair_small_holes=False)
    assert got==reference
    np.testing.assert_array_equal(depth,original)
