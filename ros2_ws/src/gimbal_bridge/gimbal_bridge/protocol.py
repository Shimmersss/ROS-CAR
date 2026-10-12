"""Gimbal MCU serial protocol v1 (docs/云台跟随设计与串口协议.md §11).

Frame: A5 5A | version | type | seq | len (u16) | payload | CRC16 (u16).
All multi-byte fields little-endian; CRC16-CCITT-FALSE over version..payload end.
"""
from dataclasses import dataclass, fields
import math
import struct

HEADER = b'\xa5\x5a'
VERSION = 1
MAX_PAYLOAD = 128
OVERHEAD = 9

STATUS, COMMAND, SYNC_REQ, SYNC_RESP, CONFIG, ACK, EVENT = range(1, 8)

MODE_DISABLE, MODE_HOLD, MODE_POSITION, MODE_RATE, MODE_HOME = range(5)
SOURCE_NONE, SOURCE_ENCODER, SOURCE_GYRO_HOMED = range(3)

FLAG_PAN_VALID, FLAG_HOMED, FLAG_MAGNET_OK, FLAG_IMU_OK = 1, 2, 4, 8
FLAG_IMU_CALIBRATING, FLAG_CMD_TIMEOUT, FLAG_PAN_LIMIT, FLAG_SERVOS_ENABLED = 16, 32, 64, 128

CONFIG_PAN_ZERO_HERE, CONFIG_PAN_LIMIT, CONFIG_TILT_MIN, CONFIG_TILT_MAX = 1, 2, 3, 4
CONFIG_GYRO_CALIBRATE, CONFIG_SAVE, CONFIG_HOME_EDGE_LEFT, CONFIG_HOME_EDGE_RIGHT = 5, 6, 7, 8

EVENT_HOME_EDGE, EVENT_MAGNET, EVENT_IMU_FAULT, EVENT_LIMIT, EVENT_CMD_TIMEOUT = range(1, 6)

NEVER_HOMED = 0xFFFFFFFF
NO_ENCODER = 0xFFFF


def crc16(data, crc=0xFFFF):
    """CRC16-CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final xor."""
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


@dataclass
class Status:
    t_us: int
    pan_source: int
    flags: int
    last_cmd_seq: int
    fault: int
    pan_rad: float
    pan_rate_radps: float
    imu_pitch_rad: float
    imu_roll_rad: float
    gyro_x: float
    gyro_y: float
    gyro_z: float
    up_x: float
    up_y: float
    up_z: float
    encoder_raw: int = NO_ENCODER
    tilt_servo_us: int = 0
    pan_servo_cmd: int = 0
    reserved: int = 0
    ms_since_home: int = NEVER_HOMED
    FORMAT = '<IBBBBffff3f3fHHhHI'

    def flag(self, bit):
        return bool(self.flags & bit)


@dataclass
class Command:
    mode: int
    reserved8: int = 0
    reserved16: int = 0
    pan_target_rad: float = math.nan
    tilt_target_rad: float = math.nan
    pan_rate_ff_radps: float = 0.
    base_yaw_rate_radps: float = math.nan
    max_pan_rate_radps: float = 0.
    max_tilt_rate_radps: float = 0.
    FORMAT = '<BBHffffff'


@dataclass
class SyncReq:
    token: int
    FORMAT = '<I'


@dataclass
class SyncResp:
    token: int
    t_rx_us: int
    t_tx_us: int
    FORMAT = '<III'


@dataclass
class Config:
    key: int
    value: float = 0.
    FORMAT = '<Bf'


@dataclass
class Ack:
    acked_type: int
    acked_seq: int
    result: int
    FORMAT = '<BBB'


@dataclass
class Event:
    t_us: int
    event: int
    value: float = 0.
    FORMAT = '<IBf'


TYPES = {STATUS: Status, COMMAND: Command, SYNC_REQ: SyncReq, SYNC_RESP: SyncResp,
         CONFIG: Config, ACK: Ack, EVENT: Event}
TYPE_OF = {cls: kind for kind, cls in TYPES.items()}
for _cls in TYPES.values():
    assert struct.calcsize(_cls.FORMAT) <= MAX_PAYLOAD


def _values(message):
    """Flatten dataclass fields in declaration order (3-vectors are three fields)."""
    return [getattr(message, f.name) for f in fields(message)]


def pack_payload(message):
    return struct.pack(type(message).FORMAT, *_values(message))


def unpack_payload(kind, payload):
    cls = TYPES.get(kind)
    if cls is None:
        raise ValueError(f'unknown frame type {kind}')
    if len(payload) != struct.calcsize(cls.FORMAT):
        raise ValueError(f'type {kind} payload length {len(payload)}')
    return cls(*struct.unpack(cls.FORMAT, payload))


def encode(message, seq):
    payload = pack_payload(message)
    body = struct.pack('<BBBH', VERSION, TYPE_OF[type(message)], seq & 0xFF, len(payload)) + payload
    return HEADER + body + struct.pack('<H', crc16(body))


class Parser:
    """Byte-stream resynchronising parser. A bad frame drops ONE byte, never the buffer."""

    def __init__(self, max_buffer=4096):
        self.buffer = bytearray()
        self.max_buffer = max_buffer
        self.crc_errors = 0
        self.format_errors = 0
        self.frames = 0

    def feed(self, data):
        self.buffer += data
        if len(self.buffer) > self.max_buffer:
            del self.buffer[:len(self.buffer)-self.max_buffer]
        out = []
        while True:
            start = self.buffer.find(HEADER)
            if start < 0:
                # Keep a trailing 0xA5 that may begin the next header.
                del self.buffer[:max(0, len(self.buffer)-1)]
                return out
            del self.buffer[:start]
            if len(self.buffer) < 7:
                return out
            version, kind, seq, length = struct.unpack_from('<BBBH', self.buffer, 2)
            if version != VERSION or length > MAX_PAYLOAD:
                self.format_errors += 1
                del self.buffer[:1]
                continue
            total = 7+length+2
            if len(self.buffer) < total:
                return out
            body = bytes(self.buffer[2:7+length])
            (received,) = struct.unpack_from('<H', self.buffer, 7+length)
            if crc16(body) != received:
                self.crc_errors += 1
                del self.buffer[:1]
                continue
            try:
                message = unpack_payload(kind, body[5:])
            except (ValueError, struct.error):
                self.format_errors += 1
                del self.buffer[:1]
                continue
            del self.buffer[:total]
            self.frames += 1
            out.append((kind, seq, message))
