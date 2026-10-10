import math
import struct
import unittest
from gimbal_bridge import protocol as p


def status(**kw):
    values = dict(t_us=123, pan_source=1, flags=p.FLAG_PAN_VALID | p.FLAG_IMU_OK, last_cmd_seq=7, fault=0,
                  pan_rad=.5, pan_rate_radps=.1, imu_pitch_rad=-.2, imu_roll_rad=.01, gyro_x=0., gyro_y=.02,
                  gyro_z=.3, up_x=.2, up_y=0., up_z=.98)
    values.update(kw)
    return p.Status(**values)


class ProtocolTests(unittest.TestCase):
    def test_crc_and_sizes_match_the_document(self):
        self.assertEqual(p.crc16(b'123456789'), 0x29B1)
        sizes = {cls.__name__: struct.calcsize(cls.FORMAT) for cls in p.TYPES.values()}
        self.assertEqual(sizes, dict(Status=60, Command=28, SyncReq=4, SyncResp=12, Config=5, Ack=3, Event=9))
        frame = p.encode(status(), 3)
        self.assertEqual(len(frame), 69)
        self.assertEqual(frame[:2], b'\xa5\x5a')
        self.assertEqual(frame[2:7], bytes([1, p.STATUS, 3, 60, 0]))
        self.assertEqual(struct.unpack('<H', frame[-2:])[0], p.crc16(frame[2:-2]))

    def test_round_trip_every_type(self):
        messages = [status(), p.Command(p.MODE_POSITION, pan_target_rad=1., tilt_target_rad=-.1, base_yaw_rate_radps=0.),
                    p.SyncReq(9), p.SyncResp(9, 1, 2), p.Config(p.CONFIG_PAN_LIMIT, 2.5), p.Ack(p.CONFIG, 4, 0),
                    p.Event(5, p.EVENT_HOME_EDGE, .01)]
        parser = p.Parser()
        out = parser.feed(b''.join(p.encode(m, i) for i, m in enumerate(messages)))
        self.assertEqual([seq for _, seq, _ in out], list(range(len(messages))))
        for (kind, _, decoded), original in zip(out, messages):
            self.assertEqual(kind, p.TYPE_OF[type(original)])
            for name, value in vars(original).items():
                got = getattr(decoded, name)
                if isinstance(value, float) and math.isnan(value):
                    self.assertTrue(math.isnan(got))
                elif isinstance(value, float):
                    self.assertAlmostEqual(got, value, places=6)
                else:
                    self.assertEqual(got, value)

    def test_resync_after_garbage_corruption_and_split_frames(self):
        good = p.encode(status(), 1)
        bad = bytearray(p.encode(status(pan_rad=9.), 2)); bad[20] ^= 0xFF
        fake_header = b'\xa5\x5a\x01\x01\x00\xff\xff'          # absurd length
        stream = b'\x00\xa5junk' + fake_header + bytes(bad) + good + good
        parser = p.Parser()
        out = []
        for i in range(0, len(stream), 7):                     # arrives in small chunks
            out += parser.feed(stream[i:i+7])
        self.assertEqual(len(out), 2)
        self.assertTrue(all(m.pan_rad == .5 for _, _, m in out))
        self.assertGreaterEqual(parser.crc_errors, 1)
        self.assertGreaterEqual(parser.format_errors, 1)

    def test_unknown_type_and_wrong_length_are_rejected(self):
        body = struct.pack('<BBBH', 1, 99, 0, 1)+b'\x00'
        parser = p.Parser()
        self.assertEqual(parser.feed(p.HEADER+body+struct.pack('<H', p.crc16(body))), [])
        body = struct.pack('<BBBH', 1, p.STATUS, 0, 4)+b'\x00'*4
        self.assertEqual(parser.feed(p.HEADER+body+struct.pack('<H', p.crc16(body))), [])
        self.assertEqual(parser.format_errors, 2)
