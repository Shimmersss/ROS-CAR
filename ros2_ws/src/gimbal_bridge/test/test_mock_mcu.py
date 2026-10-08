import math
import unittest
from gimbal_bridge import protocol as p
from gimbal_bridge.mock_mcu import MockConfig, MockMcu


def run(mcu, start, seconds, command=None, dt=.005, every=.02):
    events, t, last_cmd = [], start, -math.inf
    while t < start+seconds:
        if command is not None and t-last_cmd >= every:
            mcu.handle(p.COMMAND, 0, command, t); last_cmd = t
        events += mcu.step(dt, t); t += dt
    return t, events


class MockMcuTests(unittest.TestCase):
    def test_encoder_position_limit_and_timeout(self):
        mcu = MockMcu(MockConfig(mode='encoder'), 0.)
        status = mcu.status(0.)
        self.assertAlmostEqual(status.pan_rad, .3, delta=.002)  # absolute at power-up
        t, _ = run(mcu, 0., 2., p.Command(p.MODE_POSITION, pan_target_rad=1., tilt_target_rad=.2))
        s = mcu.status(t)
        self.assertAlmostEqual(s.pan_rad, 1., delta=.01); self.assertAlmostEqual(s.imu_pitch_rad, .2, delta=.01)
        self.assertTrue(s.flag(p.FLAG_PAN_VALID)); self.assertFalse(s.flag(p.FLAG_CMD_TIMEOUT))
        t, events = run(mcu, t, 3., p.Command(p.MODE_RATE, pan_rate_ff_radps=3.))
        self.assertAlmostEqual(mcu.status(t).pan_rad, 2.967, delta=.02)
        self.assertIn(p.EVENT_LIMIT, [e.event for e in events])
        pan = mcu.true_pan
        t, events = run(mcu, t, .5)                             # no commands: stop within 0.3 s
        self.assertTrue(mcu.status(t).flag(p.FLAG_CMD_TIMEOUT))
        self.assertEqual([e.event for e in events], [p.EVENT_CMD_TIMEOUT])
        self.assertAlmostEqual(mcu.true_pan, pan, delta=.01)

    def test_encoder_zero_and_magnet_loss(self):
        mcu = MockMcu(MockConfig(mode='encoder'), 0.)
        self.assertEqual(mcu.handle(p.CONFIG, 3, p.Config(p.CONFIG_PAN_ZERO_HERE), 0.)[0].result, 0)
        self.assertAlmostEqual(mcu.status(0.).pan_rad, 0., delta=.002)
        mcu.magnet_ok = False
        t, _ = run(mcu, 0., .2, p.Command(p.MODE_RATE, pan_rate_ff_radps=1.))
        self.assertFalse(mcu.status(t).flag(p.FLAG_PAN_VALID))
        self.assertAlmostEqual(mcu.true_pan, .3, delta=1e-9)    # never moves without a valid angle

    def test_gyro_needs_homing_and_base_rate_and_reports_drift(self):
        mcu = MockMcu(MockConfig(mode='gyro', gyro_bias=.01), 0.)
        no_rate = p.Command(p.MODE_POSITION, pan_target_rad=.5)          # base rate NaN
        t, _ = run(mcu, 0., .5, no_rate)
        self.assertFalse(mcu.status(t).flag(p.FLAG_PAN_VALID)); self.assertFalse(mcu.homed)
        t, events = run(mcu, t, 2., p.Command(p.MODE_HOME, base_yaw_rate_radps=0.))
        self.assertTrue(mcu.homed and mcu.status(t).flag(p.FLAG_PAN_VALID))
        self.assertIn(p.EVENT_HOME_EDGE, [e.event for e in events])
        t, _ = run(mcu, t, 3., p.Command(p.MODE_POSITION, pan_target_rad=.5, base_yaw_rate_radps=0.))
        self.assertAlmostEqual(mcu.status(t).pan_rad, .5, delta=.01)
        self.assertGreater(abs(mcu.true_pan-.5), .01)                     # drift: estimate != truth
        t, events = run(mcu, t, 3., p.Command(p.MODE_RATE, pan_rate_ff_radps=-.5, base_yaw_rate_radps=0.))
        drift = [e.value for e in events if e.event == p.EVENT_HOME_EDGE]
        self.assertTrue(drift and abs(drift[0]) > .01)

    def test_clock_wraps_and_sync_reply(self):
        mcu = MockMcu(MockConfig(), 0.)
        self.assertLess(mcu.mcu_us(4.), mcu.mcu_us(0.))                    # wraps 3 s after start
        reply = mcu.handle(p.SYNC_REQ, 0, p.SyncReq(42), 1.)[0]
        self.assertEqual((reply.token, reply.t_rx_us), (42, mcu.mcu_us(1.)))
        self.assertEqual(mcu.handle(p.CONFIG, 1, p.Config(99), 1.)[0].result, 1)
