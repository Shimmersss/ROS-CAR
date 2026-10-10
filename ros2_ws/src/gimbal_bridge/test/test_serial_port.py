import os
import threading
import time
import unittest

from gimbal_bridge.serial_port import write_all


class WriteAllTests(unittest.TestCase):
    def pipe(self):
        r, w = os.pipe()
        os.set_blocking(w, False)
        self.addCleanup(os.close, r)
        self.addCleanup(os.close, w)
        while True:                                   # fill the pipe so the next write is short
            try:
                os.write(w, b'x'*4096)
            except BlockingIOError:
                break
        return r, w

    def test_short_writes_complete_when_the_reader_drains(self):
        r, w = self.pipe()
        frame = bytes(range(256))*64                  # larger than one drained chunk

        def drain():
            time.sleep(.01)
            got = b''
            while not got.endswith(frame[-64:]):
                got += os.read(r, 65536)
        reader = threading.Thread(target=drain)
        reader.start()
        write_all(w, frame, 1.)
        reader.join(2.)
        self.assertFalse(reader.is_alive())

    def test_stalled_port_times_out(self):
        _, w = self.pipe()
        start = time.monotonic()
        with self.assertRaises(TimeoutError):
            write_all(w, b'frame', .05)
        self.assertLess(time.monotonic()-start, .5)


if __name__ == '__main__':
    unittest.main()
