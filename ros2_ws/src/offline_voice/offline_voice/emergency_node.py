"""Independent streaming KWS: never gates on wake/session/ASR/TTS state."""
import json
import os
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from std_msgs.msg import Bool, String
from roscar_interfaces.msg import AudioFrame
from ament_index_python.packages import get_package_share_directory
from .frontend import EmergencySpotter


class EmergencyKwsNode(Node):
    def __init__(self):
        super().__init__('emergency_kws')
        model_root = Path(os.environ.get('ROSCAR_OFFLINE_MODEL_ROOT', '/home/wheeltec/ROSCAR-offline/models'))
        self.declare_parameter('model_dir', str(model_root / 'sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20'))
        self.declare_parameter('keywords_file', str(Path(get_package_share_directory('offline_voice')) / 'config/emergency_keywords.txt'))
        self.healthy_pub = self.create_publisher(Bool, '/voice/emergency_healthy', 10)
        self.stop_pub = self.create_publisher(Bool, '/control/emergency_stop', 10)
        self.events = self.create_publisher(String, '/voice/emergency_event', 10)
        self.engine = None
        self.failed = False
        try:
            self.engine = EmergencySpotter(self.get_parameter('model_dir').value,
                                            self.get_parameter('keywords_file').value)
        except Exception as exc:
            self.failed = True
            self.get_logger().error(f'Emergency KWS unavailable: {exc}; motion must remain disarmed')
        self.last_audio = -float('inf')
        self.sequence = None
        self.pending_stop = False
        self.estop_seen = False
        self.create_subscription(AudioFrame, '/voice/audio', self.on_audio, 20)
        self.create_subscription(String, '/control/state', self.on_state, 10)
        self.timer = self.create_timer(.1, self.heartbeat, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_state(self, message):
        try:
            state = json.loads(message.data)['authority_state']
            if state == 'ESTOP':
                self.estop_seen = True
                self.pending_stop = False
            elif state == 'DISARMED' and self.estop_seen:
                self.estop_seen = False
                self.pending_stop = False
                if self.engine:
                    self.engine.reset()
        except (ValueError, TypeError, KeyError):
            pass

    def on_audio(self, message):
        if self.failed:
            return
        try:
            age = self.get_clock().now().nanoseconds * 1e-9 - (message.stamp.sec + message.stamp.nanosec * 1e-9)
            if (message.sample_rate != 16000 or message.channels != 1 or len(message.pcm_s16le) != 1280
                    or not 0 <= age <= .2 or (self.sequence is not None and message.sequence != self.sequence + 1)):
                raise ValueError('Invalid/stale/gapped emergency audio')
            keyword = self.engine.accept(np.frombuffer(bytes(message.pcm_s16le), dtype='<i2').astype(np.float32) / 32768.)
            self.sequence, self.last_audio = message.sequence, time.monotonic()
            if keyword:
                self.pending_stop = True
                self.stop_pub.publish(Bool(data=True))
                self.events.publish(String(data=json.dumps({'keyword': keyword, 'audio_sequence': message.sequence,
                    'detected_at': self.get_clock().now().nanoseconds * 1e-9}, ensure_ascii=False)))
        except Exception as exc:
            self.failed = True
            self.healthy_pub.publish(Bool(data=False))
            self.stop_pub.publish(Bool(data=True))
            self.get_logger().error(f'Emergency channel failed: {exc}')

    def heartbeat(self):
        self.healthy_pub.publish(Bool(data=not self.failed and time.monotonic() - self.last_audio <= .2))
        if self.pending_stop:
            self.stop_pub.publish(Bool(data=True))

    def destroy_node(self):
        self.healthy_pub.publish(Bool(data=False))
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EmergencyKwsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
