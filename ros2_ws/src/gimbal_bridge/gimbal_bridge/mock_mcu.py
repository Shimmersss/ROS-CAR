"""Gimbal MCU simulator for bench tests without hardware; also a behavioural reference
for the firmware (timeouts, limits, homing, flags). Not a model of real servo dynamics."""
import argparse
from dataclasses import dataclass
import math
import os
import select
import signal
import threading
import time
import tty

from . import protocol as p

TWO_PI = 2*math.pi


def wrap_pi(angle):
    return (angle+math.pi) % TWO_PI-math.pi


@dataclass
class MockConfig:
    mode: str = 'encoder'                  # 'encoder' (scheme E) or 'gyro' (scheme G)
    clock_start_us: int = (1 << 32)-3_000_000   # MCU counter wraps 3 s after start
    drift_ppm: float = 40.
    pan_rate_max: float = 2.
    tilt_rate_max: float = 1.
    pan_gain: float = 6.
    pan_limit: float = 2.967
    tilt_min: float = -.524
    tilt_max: float = .785
    cmd_timeout_s: float = .3
    base_rate_timeout_s: float = .1
    imu_cal_s: float = .3
    gyro_bias: float = .002
    home_rate: float = .5
    home_timeout_s: float = 15.
    initial_pan: float = .3
    encoder_mount_raw: int = 1234          # raw reading at true pan 0 before zeroing
    status_period_s: float = .005

    def __post_init__(self):
        if self.mode not in ('encoder', 'gyro'):
            raise ValueError("mode must be 'encoder' or 'gyro'")


class MockMcu:
    """Pure simulation: feed frames with handle(), advance with step(), read status()."""

    def __init__(self, config=None, now=0.):
        self.cfg = config or MockConfig()
        self.t0 = now
        self.true_pan = self.cfg.initial_pan
        self.pan_est = 0.
        self.homed = False
        self.home_at = None
        self.homing_since = None
        self.zero_raw = self.cfg.encoder_mount_raw if self.cfg.mode == 'encoder' else 0
        self.tilt = 0.
        self.tilt_rate = 0.
        self.pan_rate = 0.
        self.command = None
        self.command_at = -math.inf
        self.command_seq = 0
        self.ever_commanded = False
        self.timeout_reported = False
        self.limit_reported = False
        self.magnet_ok = True
        self.pan_limit_rad = self.cfg.pan_limit
        self.tilt_min, self.tilt_max = self.cfg.tilt_min, self.cfg.tilt_max
        self.flags = 0

    # --- clock -----------------------------------------------------------------
    def mcu_us(self, now):
        elapsed = (now-self.t0)*(1+self.cfg.drift_ppm*1e-6)
        return int(self.cfg.clock_start_us+elapsed*1e6) & 0xFFFFFFFF

    # --- encoder ---------------------------------------------------------------
    def encoder_raw(self):
        return int(round(self.true_pan/TWO_PI*4096+self.cfg.encoder_mount_raw)) % 4096

    def _measured_pan(self):
        if self.cfg.mode == 'encoder':
            counts = (self.encoder_raw()-self.zero_raw+2048) % 4096-2048
            return counts*TWO_PI/4096
        return self.pan_est

    def _pan_valid(self, now):
        if self.cfg.mode == 'encoder':
            return self.magnet_ok
        fresh = (self.command is not None and now-self.command_at <= self.cfg.base_rate_timeout_s
                 and math.isfinite(self.command.base_yaw_rate_radps))
        return self.homed and fresh

    # --- input -----------------------------------------------------------------
    def handle(self, kind, seq, message, now):
        """Return response messages for one received frame."""
        if kind == p.COMMAND:
            if message.mode > p.MODE_HOME:
                return []
            if message.mode == p.MODE_HOME and (self.command is None or self.command.mode != p.MODE_HOME):
                self.homing_since = now
            self.command, self.command_at, self.command_seq = message, now, seq
            self.ever_commanded = True
            self.timeout_reported = False
            return []
        if kind == p.SYNC_REQ:
            stamp = self.mcu_us(now)
            return [p.SyncResp(message.token, stamp, stamp)]
        if kind == p.CONFIG:
            ok = self._config(message)
            return [p.Ack(p.CONFIG, seq, 0 if ok else 1)]
        return []

    def _config(self, message):
        key, value = message.key, message.value
        if key == p.CONFIG_PAN_ZERO_HERE and self.cfg.mode == 'encoder' and self.magnet_ok:
            self.zero_raw = self.encoder_raw()
        elif key == p.CONFIG_PAN_LIMIT and 0 < value <= math.pi:
            self.pan_limit_rad = value
        elif key == p.CONFIG_TILT_MIN and value < self.tilt_max:
            self.tilt_min = value
        elif key == p.CONFIG_TILT_MAX and value > self.tilt_min:
            self.tilt_max = value
        elif key in (p.CONFIG_GYRO_CALIBRATE, p.CONFIG_SAVE):
            pass
        else:
            return False
        return True

    # --- dynamics --------------------------------------------------------------
    def step(self, dt, now):
        """Advance the simulation; returns Event messages."""
        cfg, events = self.cfg, []
        fresh = self.command is not None and now-self.command_at <= cfg.cmd_timeout_s
        if not fresh and self.ever_commanded and not self.timeout_reported:
            events.append(p.Event(self.mcu_us(now), p.EVENT_CMD_TIMEOUT, (now-self.command_at)*1000.))
            self.timeout_reported = True
        command = self.command if fresh else None
        mode = command.mode if command else p.MODE_HOLD
        valid = self._pan_valid(now)
        pan = self._measured_pan()
        max_rate = cfg.pan_rate_max
        if command and command.max_pan_rate_radps > 0:
            max_rate = min(max_rate, command.max_pan_rate_radps)
        rate = 0.
        if mode == p.MODE_POSITION and valid and math.isfinite(command.pan_target_rad):
            target = max(-self.pan_limit_rad, min(self.pan_limit_rad, command.pan_target_rad))
            rate = cfg.pan_gain*(target-pan)+(command.pan_rate_ff_radps or 0.)
        elif mode == p.MODE_RATE and valid:
            rate = command.pan_rate_ff_radps
        elif mode == p.MODE_HOME:
            if cfg.mode == 'encoder' and valid:
                rate = cfg.pan_gain*(0.-pan)
            elif cfg.mode == 'gyro' and not self.homed and now-(self.homing_since or now) <= cfg.home_timeout_s:
                rate = -cfg.home_rate
        rate = max(-max_rate, min(max_rate, rate))
        # Soft limit is enforced here, independent of the host.
        at_limit = valid and ((pan >= self.pan_limit_rad and rate > 0) or (pan <= -self.pan_limit_rad and rate < 0))
        if at_limit:
            rate = 0.
            if not self.limit_reported:
                events.append(p.Event(self.mcu_us(now), p.EVENT_LIMIT, pan))
                self.limit_reported = True
        elif valid and abs(pan) < self.pan_limit_rad-.05:
            self.limit_reported = False
        previous = self.true_pan
        self.true_pan += rate*dt
        self.pan_rate = rate
        if cfg.mode == 'gyro':
            self.pan_est += (rate+cfg.gyro_bias)*dt
            if previous*self.true_pan <= 0 and previous != self.true_pan:
                events.append(p.Event(self.mcu_us(now), p.EVENT_HOME_EDGE, self.pan_est if self.homed else math.nan))
                self.pan_est, self.homed, self.home_at = 0., True, now
        tilt_target = self.tilt
        if command and mode in (p.MODE_POSITION, p.MODE_RATE, p.MODE_HOME) and math.isfinite(command.tilt_target_rad):
            tilt_target = max(self.tilt_min, min(self.tilt_max, command.tilt_target_rad))
        tilt_max_rate = cfg.tilt_rate_max
        if command and command.max_tilt_rate_radps > 0:
            tilt_max_rate = min(tilt_max_rate, command.max_tilt_rate_radps)
        delta = max(-tilt_max_rate*dt, min(tilt_max_rate*dt, tilt_target-self.tilt))
        self.tilt += delta
        self.tilt_rate = delta/dt if dt > 0 else 0.
        self.flags = 0
        for condition, bit in ((self._pan_valid(now), p.FLAG_PAN_VALID), (self.homed, p.FLAG_HOMED),
                               (cfg.mode == 'encoder' and self.magnet_ok, p.FLAG_MAGNET_OK),
                               (True, p.FLAG_IMU_OK), (now-self.t0 < cfg.imu_cal_s, p.FLAG_IMU_CALIBRATING),
                               (not fresh, p.FLAG_CMD_TIMEOUT), (at_limit, p.FLAG_PAN_LIMIT),
                               (mode != p.MODE_DISABLE, p.FLAG_SERVOS_ENABLED)):
            if condition:
                self.flags |= bit
        return events

    def status(self, now):
        pitch = self.tilt
        gyro_z = self.pan_rate+(self.cfg.gyro_bias if self.cfg.mode == 'gyro' else 0.)
        since = (p.NEVER_HOMED if self.cfg.mode == 'encoder' or self.home_at is None
                 else min(int((now-self.home_at)*1000), p.NEVER_HOMED-1))
        return p.Status(
            self.mcu_us(now), p.SOURCE_ENCODER if self.cfg.mode == 'encoder' else p.SOURCE_GYRO_HOMED,
            self.flags, self.command_seq, 0, self._measured_pan(), self.pan_rate, pitch, 0.,
            0., self.tilt_rate, gyro_z, -math.sin(pitch), 0., math.cos(pitch),
            self.encoder_raw() if self.cfg.mode == 'encoder' else p.NO_ENCODER,
            int(1500+pitch/(math.pi/2)*1000), int(max(-1000, min(1000, self.pan_rate/self.cfg.pan_rate_max*1000))),
            0, since)


class PtyMock:
    """Runs MockMcu on a pseudo-terminal; the bridge opens `slave_path` (or the symlink)."""

    def __init__(self, config=None, link=None, corrupt_every=0):
        self.master, slave = os.openpty()
        tty.setraw(slave)
        # Never block the simulator when no bridge is reading yet.
        os.set_blocking(self.master, False)
        self.slave_fd = slave
        self.slave_path = os.ttyname(slave)
        self.link = link
        if link:
            if os.path.lexists(link):
                os.unlink(link)
            os.symlink(self.slave_path, link)
        self.mcu = MockMcu(config, time.monotonic())
        self.parser = p.Parser()
        self.corrupt_every = corrupt_every
        self.sent = 0
        self.seq = {}
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.thread = None

    def _send(self, message):
        kind = p.TYPE_OF[type(message)]
        self.seq[kind] = (self.seq.get(kind, -1)+1) & 0xFF
        frame = bytearray(p.encode(message, self.seq[kind]))
        self.sent += 1
        if self.corrupt_every and self.sent % self.corrupt_every == 0:
            frame[len(frame)//2] ^= 0x5A
        try:
            os.write(self.master, bytes(frame))
        except (BlockingIOError, OSError):
            pass

    def run(self):
        last = time.monotonic()
        next_status = last
        while not self.stop_event.is_set():
            ready, _, _ = select.select([self.master], [], [], max(0., next_status-time.monotonic()))
            now = time.monotonic()
            with self.lock:
                if ready:
                    try:
                        data = os.read(self.master, 4096)
                    except OSError:
                        data = b''
                    for kind, seq, message in self.parser.feed(data):
                        for response in self.mcu.handle(kind, seq, message, time.monotonic()):
                            self._send(response)
                if now >= next_status:
                    for event in self.mcu.step(now-last, now):
                        self._send(event)
                    last = now
                    self._send(self.mcu.status(now))
                    next_status = now+self.mcu.cfg.status_period_s

    def start(self):
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()
        return self

    def close(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=1.)
        for fd in (self.master, self.slave_fd):
            try:
                os.close(fd)
            except OSError:
                pass
        if self.link and os.path.islink(self.link):
            os.unlink(self.link)


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--link', default='/tmp/roscar_gimbal_mock', help='symlink to the pty slave')
    parser.add_argument('--mode', choices=('encoder', 'gyro'), default='encoder')
    parser.add_argument('--corrupt-every', type=int, default=0, help='flip a byte in every Nth frame')
    parser.add_argument('--drift-ppm', type=float, default=40.)
    parsed, _ = parser.parse_known_args(args)   # tolerate --ros-args from launch
    mock = PtyMock(MockConfig(mode=parsed.mode, drift_ppm=parsed.drift_ppm), parsed.link,
                   parsed.corrupt_every).start()
    print(f'mock gimbal MCU ({parsed.mode}) on {mock.slave_path} -> {parsed.link}', flush=True)
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    stop.wait()
    mock.close()


if __name__ == '__main__':
    main()
