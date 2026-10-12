import numpy as np
from yolo_person_tracker.registration import depth_millimetres


def test_mm_invalid_and_overflow_do_not_wrap_into_real_ranges():
    result = depth_millimetres(np.array([np.nan, np.inf, -np.inf, -1., 0., 65.536, 1.2344, 8.]))
    assert result.dtype == np.uint16
    np.testing.assert_array_equal(result, [0, 0, 0, 0, 0, 0, 1234, 8000])


def test_camera_range_round_trip_error_is_at_most_half_a_millimetre():
    depth = np.linspace(.2, 8., 10001, dtype=np.float32)
    recovered = depth_millimetres(depth).astype(np.float64) * .001
    assert np.max(np.abs(recovered - depth)) <= .00050001
