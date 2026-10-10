"""Serial <-> ROS bridge for the gimbal MCU. Publishes joint states only with a valid,
time-synchronised source; missing data removes the TF instead of holding stale angles."""
import math
import os
import select
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from std_srvs.srv import Trigger
from gimbal_interfaces.msg import GimbalCommand, GimbalStatus

from . import protocol as p
from .serial_port import open_port, write_all
from .timesync import ClockSync, Unwrapper

GRAVITY = 9.80665


WRITE_TIMEOUT_S = .02   # one 50 Hz command period


def to_stamp(seconds):
    sec = math.floor(seconds)
    nanosec = int(round((seconds-sec)*1e9))
    if nanosec >= 1000000000:
        sec, nanosec = sec+1, nanosec-1000000000
    return Time(seconds=sec, nanoseconds=nanosec).to_msg()


class GimbalBridge(Node):
    def __init__(self, **kwargs):
        super().__init__('gimbal_bridge', **kwargs)
        defaults = dict(
            port='', baud=460800, pan_joint='pan_joint', tilt_joint='tilt_joint', imu_frame='imu_link',
            command_rate_hz=50., command_timeout_s=.2, status_timeout_s=.05, sync_period_s=1.,
            max_sync_rtt_s=.02, max_crc_error_ratio=.05, imu_pitch_offset_rad=0., base_pitch_rad=0.,
            # 'odom': chassis yaw rate from odom_topic; 'zero': bench only, chassis known still.
            base_yaw_rate_source='odom', odom_topic='/odom', odom_timeout_s=.2,
            max_pan_rate_radps=.785, max_tilt_rate_radps=.524)
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.cfg = {name: self.get_parameter(name).value for name in defaults}
        c = self.cfg
        if c['base_yaw_rate_source'] not in ('odom', 'zero'):
            raise ValueError("base_yaw_rate_source must be 'odom' or 'zero'")
        for name in ('command_rate_hz', 'command_timeout_s', 'status_timeout_s', 'sync_period_s',
                     'max_sync_rtt_s', 'odom_timeout_s'):
            if not math.isfinite(c[name]) or c[name] <= 0:
                raise ValueError(f'{name} must be finite and positive')
        self.lock = threading.RLock()
        self.write_lock = threading.Lock()
        self.fd = None
        self.parser = p.Parser()
        self.unwrap = Unwrapper()
        self.sync = ClockSync(max_rtt_s=c['max_sync_rtt_s'])
        self.seq = {}
        self.pending_sync = {}
        self.token = 0
        self.acks = {}
        self.command = None
        self.command_at = -math.inf
        self.odom_rate = math.nan
        self.odom_at = -math.inf
        self.last_status = None
        self.last_status_at = -math.inf
        self.last_joint_stamp = -math.inf
        self.last_event = ''
        self.crc_window = (time.monotonic(), 0, 0)
        self.crc_ok = True
        self.stop_event = threading.Event()
        self.status_pub = self.create_publisher(GimbalStatus, 'gimbal/status', 10)
        self.joint_pub = self.create_publisher(JointState, 'gimbal/joint_states', 10)
        self.imu_pub = self.create_publisher(Imu, 'gimbal/imu', 10)
        self.create_subscription(GimbalCommand, 'gimbal/command', self.on_command, 10)
        if c['base_yaw_rate_source'] == 'odom':
            self.create_subscription(Odometry, c['odom_topic'], self.on_odom, 10)
        else:
            self.get_logger().warning("base_yaw_rate_source='zero': bench use only, chassis must not rotate")
        for name, key in (('set_pan_zero', p.CONFIG_PAN_ZERO_HERE), ('calibrate_gyro', p.CONFIG_GYRO_CALIBRATE),
                          ('save_config', p.CONFIG_SAVE)):
            self.create_service(Trigger, f'gimbal/{name}', lambda req, res, key=key: self.config_service(key, res))
        self.create_timer(1./c['command_rate_hz'], self.send_command)
        self.create_timer(c['sync_period_s'], self.send_sync)
        self.create_timer(.05, self.watchdog)
        self.create_timer(1., self.ensure_open)
        self.reader = threading.Thread(target=self.read_loop, daemon=True)
        self.reader.start()
        self.ensure_open()

    # --- port -----------------------------------------------------------------
    def ensure_open(self):
        with self.lock:
            if self.fd is not None or not self.cfg['port']:
                return
            try:
                self.fd = open_port(self.cfg['port'], self.cfg['baud'])
            except (OSError, ValueError) as exc:
                self.last_event = f'port unavailable: {exc}'
                return
            self.parser = p.Parser()
            self.unwrap = Unwrapper()
            self.sync.reset()
            self.pending_sync.clear()
            self.get_logger().info(f'opened {self.cfg["port"]}')
        self.send_sync()

    def close_port(self, reason):
        with self.lock:
            if self.fd is not None:
                try:
                    os.close(self.fd)
                except OSError:
                    pass
            self.fd = None
            self.last_status = None
            self.sync.reset()
            self.last_event = reason
        self.get_logger().warning(f'serial closed: {reason}')

    def write(self, message):
        kind = p.TYPE_OF[type(message)]
        with self.write_lock:
            self.seq[kind] = (self.seq.get(kind, -1)+1) & 0xFF
            seq = self.seq[kind]
            fd = self.fd
            if fd is None:
                return None
            try:
                # A whole frame or nothing usable: a stalled port is closed and reopened.
                write_all(fd, p.encode(message, seq), WRITE_TIMEOUT_S)
            except (TimeoutError, OSError) as exc:
                self.close_port(f'write failed: {exc}')
                return None
        return seq

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9

    # --- reader thread -------------------------------------------------------------
    def read_loop(self):
        while not self.stop_event.is_set():
            fd = self.fd
            if fd is None:
                time.sleep(.05)
                continue
            try:
                ready, _, _ = select.select([fd], [], [], .05)
                if not ready:
                    continue
                data = os.read(fd, 4096)
                received = self.now()
            except (OSError, ValueError) as exc:
                if fd == self.fd:
                    self.close_port(f'read failed: {exc}')
                continue
            if not data:
                continue
            with self.lock:
                frames = self.parser.feed(data)
                for kind, seq, message in frames:
                    self.handle_frame(kind, seq, message, received)

    def handle_frame(self, kind, seq, message, received):
        if kind == p.STATUS:
            self.on_status(message, received)
        elif kind == p.SYNC_RESP:
            sent = self.pending_sync.pop(message.token, None)
            if sent is not None:
                t_rx, t_tx = self.unwrap(message.t_rx_us), self.unwrap(message.t_tx_us)
                self.sync.add(sent, t_rx, t_tx, received)
        elif kind == p.ACK:
            waiter = self.acks.get(message.acked_seq)
            if waiter is not None and message.acked_type == p.CONFIG:
                waiter[1] = message.result
                waiter[0].set()
        elif kind == p.EVENT:
            names = {p.EVENT_HOME_EDGE: 'home edge, drift', p.EVENT_MAGNET: 'magnet status',
                     p.EVENT_IMU_FAULT: 'IMU fault', p.EVENT_LIMIT: 'pan limit at',
                     p.EVENT_CMD_TIMEOUT: 'command timeout after ms'}
            self.last_event = f'{names.get(message.event, "event")} {message.value:.4g}'
            self.get_logger().info(f'MCU event: {self.last_event}')

    def update_crc_health(self):
        start, crc0, ok0 = self.crc_window
        now = time.monotonic()
        if now-start >= 1.:
            bad, good = self.parser.crc_errors-crc0, self.parser.frames-ok0
            self.crc_ok = bad <= self.cfg['max_crc_error_ratio']*max(1, bad+good)
            self.crc_window = (now, self.parser.crc_errors, self.parser.frames)

    def on_status(self, s, received):
        self.update_crc_health()
        mcu_time = self.unwrap(s.t_us)
        stamp = self.sync.to_host(mcu_time)
        self.last_status, self.last_status_at = s, time.monotonic()
        msg = self.status_message(s, stamp if stamp is not None else received)
        self.status_pub.publish(msg)
        # Never publish joint angles on the host receive clock: TF lookups are by sample time.
        if stamp is None or stamp <= self.last_joint_stamp:
            return
        joints = JointState()
        joints.header.stamp = to_stamp(stamp)
        if s.flag(p.FLAG_IMU_OK) and not s.flag(p.FLAG_IMU_CALIBRATING):
            joints.name.append(self.cfg['tilt_joint'])
            joints.position.append(s.imu_pitch_rad-self.cfg['base_pitch_rad']-self.cfg['imu_pitch_offset_rad'])
            imu = Imu()
            imu.header.stamp = joints.header.stamp
            imu.header.frame_id = self.cfg['imu_frame']
            imu.orientation_covariance[0] = -1.   # orientation not provided
            imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = s.gyro_x, s.gyro_y, s.gyro_z
            imu.linear_acceleration.x = s.up_x*GRAVITY
            imu.linear_acceleration.y = s.up_y*GRAVITY
            imu.linear_acceleration.z = s.up_z*GRAVITY
            self.imu_pub.publish(imu)
        if s.flag(p.FLAG_PAN_VALID) and self.crc_ok and math.isfinite(s.pan_rad):
            joints.name.append(self.cfg['pan_joint'])
            joints.position.append(s.pan_rad)
        if joints.name:
            self.last_joint_stamp = stamp
            self.joint_pub.publish(joints)

    def status_message(self, s, stamp):
        msg = GimbalStatus()
        msg.header.stamp = to_stamp(stamp)
        msg.connected = s is not None and self.fd is not None
        msg.time_synced = self.sync.synced
        msg.frames_ok, msg.crc_errors = self.parser.frames, self.parser.crc_errors
        msg.clock_offset_s = float(self.sync.offset())
        msg.sync_rtt_s = float(self.sync.best_rtt) if math.isfinite(self.sync.best_rtt) else math.nan
        msg.clock_steps = self.sync.steps
        msg.detail = self.last_event
        if s is None:
            msg.pan_rad = msg.pan_rate_radps = msg.imu_pitch_rad = msg.imu_roll_rad = math.nan
            return msg
        msg.pan_source, msg.fault = s.pan_source, s.fault
        msg.pan_valid = s.flag(p.FLAG_PAN_VALID) and self.crc_ok
        msg.homed, msg.magnet_ok = s.flag(p.FLAG_HOMED), s.flag(p.FLAG_MAGNET_OK)
        msg.imu_ok, msg.imu_calibrating = s.flag(p.FLAG_IMU_OK), s.flag(p.FLAG_IMU_CALIBRATING)
        msg.cmd_timeout, msg.pan_limit = s.flag(p.FLAG_CMD_TIMEOUT), s.flag(p.FLAG_PAN_LIMIT)
        msg.servos_enabled = s.flag(p.FLAG_SERVOS_ENABLED)
        msg.pan_rad, msg.pan_rate_radps = s.pan_rad, s.pan_rate_radps
        msg.imu_pitch_rad, msg.imu_roll_rad = s.imu_pitch_rad, s.imu_roll_rad
        msg.ms_since_home = s.ms_since_home
        if not self.crc_ok:
            msg.detail = 'CRC error ratio too high; pan withheld'
        return msg

    # --- host -> MCU -------------------------------------------------------------
    def on_command(self, msg):
        if msg.mode > GimbalCommand.HOME:
            self.get_logger().warning(f'ignored unknown gimbal mode {msg.mode}')
            return
        with self.lock:
            self.command, self.command_at = msg, time.monotonic()

    def on_odom(self, msg):
        with self.lock:
            self.odom_rate, self.odom_at = float(msg.twist.twist.angular.z), time.monotonic()

    def base_yaw_rate(self):
        if self.cfg['base_yaw_rate_source'] == 'zero':
            return 0.
        fresh = time.monotonic()-self.odom_at <= self.cfg['odom_timeout_s']
        return self.odom_rate if fresh and math.isfinite(self.odom_rate) else math.nan

    def send_command(self):
        with self.lock:
            msg = self.command if time.monotonic()-self.command_at <= self.cfg['command_timeout_s'] else None
            base_rate = self.base_yaw_rate()
        c = self.cfg
        if msg is None:
            # Keep the link alive in a safe mode; the MCU still times out if the bridge dies.
            command = p.Command(p.MODE_HOLD, base_yaw_rate_radps=base_rate)
        else:
            max_pan = msg.max_pan_rate_radps if msg.max_pan_rate_radps > 0 else c['max_pan_rate_radps']
            max_tilt = msg.max_tilt_rate_radps if msg.max_tilt_rate_radps > 0 else c['max_tilt_rate_radps']
            command = p.Command(msg.mode, pan_target_rad=msg.pan_target_rad, tilt_target_rad=msg.tilt_target_rad,
                                pan_rate_ff_radps=msg.pan_rate_ff_radps, base_yaw_rate_radps=base_rate,
                                max_pan_rate_radps=min(max_pan, c['max_pan_rate_radps']),
                                max_tilt_rate_radps=min(max_tilt, c['max_tilt_rate_radps']))
        self.write(command)

    def send_sync(self):
        with self.lock:
            self.token = (self.token+1) & 0xFFFFFFFF
            token = self.token
            now = self.now()
            self.pending_sync = {k: v for k, v in self.pending_sync.items() if now-v < 1.}
            self.pending_sync[token] = now
        self.write(p.SyncReq(token))

    def config_service(self, key, response):
        event = threading.Event()
        waiter = [event, None]
        with self.lock:
            seq = (self.seq.get(p.CONFIG, -1)+1) & 0xFF
            self.acks[seq] = waiter
        sent = self.write(p.Config(key, 0.))
        ok = sent is not None and event.wait(.5) and waiter[1] == 0
        with self.lock:
            self.acks.pop(seq, None)
        response.success = bool(ok)
        response.message = 'acknowledged' if ok else ('no port' if sent is None else 'rejected or no ACK')
        return response

    def watchdog(self):
        with self.lock:
            stale = time.monotonic()-self.last_status_at > self.cfg['status_timeout_s']
            if stale and self.last_status is not None:
                self.last_status = None
                self.last_event = 'status timeout'
            if stale:
                msg = self.status_message(None, self.now())
                msg.connected = False
                msg.detail = self.last_event or 'waiting for MCU status'
                self.status_pub.publish(msg)

    def destroy_node(self):
        self.stop_event.set()
        self.reader.join(timeout=1.)
        if self.fd is not None:
            os.close(self.fd)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GimbalBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
