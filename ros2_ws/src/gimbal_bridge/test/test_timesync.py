import math
import random
import unittest
from gimbal_bridge.timesync import ClockSync, Unwrapper, WRAP


class TimeSyncTests(unittest.TestCase):
    def test_unwrap_forward_and_late_samples(self):
        u = Unwrapper()
        near = WRAP-1000
        self.assertAlmostEqual(u(near), near*1e-6)
        self.assertAlmostEqual(u(500), (WRAP+500)*1e-6)       # wrapped forward
        self.assertAlmostEqual(u(WRAP-10), (WRAP-10)*1e-6)    # late sample from before the wrap
        self.assertAlmostEqual(u(2000), (WRAP+2000)*1e-6)

    def test_offset_and_drift_with_jitter(self):
        rng = random.Random(1)
        offset, drift = 1234.5, 40e-6
        sync = ClockSync(max_rtt_s=.02)
        for i in range(12):
            host_send = 100.+i
            up, down, turnaround = rng.uniform(.0005, .004), rng.uniform(.0005, .004), .0001
            mcu_rx = (host_send+up-offset)/(1+drift)
            mcu_tx = mcu_rx+turnaround
            host_recv = host_send+up+turnaround+down
            sync.add(host_send, mcu_rx, mcu_tx, host_recv)
        self.assertTrue(sync.synced)
        for host in (105., 111.5):
            mcu = (host-offset)/(1+drift)
            self.assertLess(abs(sync.to_host(mcu)-host), .002)

    def test_needs_low_rtt_samples_and_resets_on_host_rewind(self):
        sync = ClockSync(max_rtt_s=.02, min_samples=3)
        for i in range(5):
            sync.add(10.+i, 5.+i, 5.+i, 10.+i+.05)              # 50 ms RTT: too slow
        self.assertFalse(sync.synced)
        for i in range(3):
            sync.add(20.+i, 15.+i, 15.+i, 20.+i+.002)
        self.assertTrue(sync.synced)
        sync.add(1., 15., 15., 1.002)                           # host clock went backwards
        self.assertTrue(sync.synced and sync.steps == 1)        # re-anchored on the new timeline
        self.assertAlmostEqual(sync.to_host(16.), 2.001, places=6)
        self.assertFalse(sync.add(30., 20., 19., 31.))           # MCU tx before rx: rejected
        self.assertTrue(math.isnan(ClockSync().offset()))


class ClockStepTests(unittest.TestCase):
    def test_forward_host_step_re_anchors_without_dropping_sync(self):
        sync = ClockSync(max_rtt_s=.02)
        for i in range(5):
            sync.add(100.+i, 50.+i, 50.+i, 100.+i+.002)
        self.assertAlmostEqual(sync.to_host(55.), 105.001, places=4)
        sync.add(105.164, 55., 55., 105.166)                    # host stepped +164 ms
        self.assertTrue(sync.synced); self.assertEqual(sync.steps, 1)
        self.assertAlmostEqual(sync.to_host(56.), 106.165, places=4)
        sync.add(106.2, 56., 56., 106.25)                       # slow exchange: no re-anchor
        self.assertEqual(sync.steps, 1)
        self.assertTrue(sync.synced)                            # still anchored, no publishing gap
        self.assertAlmostEqual(sync.to_host(57.), 107.165, places=4)
        for i in range(3):
            sync.add(107.164+i, 57.+i, 57.+i, 107.166+i)
        self.assertFalse(sync.anchored); self.assertAlmostEqual(sync.to_host(60.), 110.165, places=4)
