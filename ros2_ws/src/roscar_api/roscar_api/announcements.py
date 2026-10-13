"""Observe stage transitions and call /voice/say without blocking perception or control."""
from collections import OrderedDict
import json
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from person_interfaces.msg import OwnerState
from roscar_interfaces.srv import SayText
from std_msgs.msg import String
from .phases import PhaseTransitions


class PhaseAnnouncements(Node):
    def __init__(self, **kwargs):
        super().__init__('phase_announcements', **kwargs)
        self.transitions = PhaseTransitions()
        self.pending = OrderedDict()
        self.current = {}
        self.inflight = None
        self.inflight_started = 0.
        self.retry_at = 0.
        self.client = self.create_client(SayText, '/voice/say')
        self.events = self.create_publisher(String, '/system/phase', 10)
        self.create_subscription(OwnerState, '/perception/owner_state', self.on_owner, 10)
        self.create_subscription(String, '/control/state', self.on_control, 10)
        self.create_timer(.1, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def enqueue(self, source, phase, text):
        now = time.monotonic()
        self.current[source] = (phase, now)
        if text:
            self.pending[source] = (phase, text, now)
            self.events.publish(String(data=json.dumps(dict(source=source, phase=phase, text=text), ensure_ascii=False)))
        elif source in self.pending:
            item = self.pending[source]
            if item[0] == phase:
                # Keep a still-current announcement through TTS startup; heartbeat
                # refreshes freshness only, it never creates another utterance.
                self.pending[source] = (phase, item[1], now)
            else:
                self.pending.pop(source)

    def on_owner(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        if stamp <= 0 or not 0 <= self.get_clock().now().nanoseconds*1e-9-stamp <= 1.:
            return
        self.enqueue('owner', msg.state, self.transitions.owner(msg.state, msg.event, msg.sequence))

    def on_control(self, msg):
        try:
            state = json.loads(msg.data)
            if isinstance(state, dict):
                phase = str(state.get('command_mode', '')) + ':' + str(state.get('authority_state', ''))
                self.enqueue('control', phase, self.transitions.control(state))
        except (ValueError, TypeError):
            return

    def tick(self):
        # At most one own service request. Coalesce unsent transitions per source;
        # an unavailable/busy TTS must never build an unbounded stale speech queue.
        now = time.monotonic()
        if self.inflight is not None:
            source, item, future = self.inflight
            if not future.done():
                if now-self.inflight_started < 130.:
                    return
                # The bridge normally bounds playback at 120 s. A dead service must
                # not wedge this observer forever; this never cancels global speech.
                future.cancel()
                self.get_logger().warning('阶段播报接口未响应，丢弃本次请求')
            self.inflight = None
            try:
                result = future.result() if not future.cancelled() else None
                if result is None:
                    raise RuntimeError('Speech service did not return a result')
                current = self.current.get(source)
                if (result.status in ('busy', 'unavailable') and source not in self.pending
                        and current and current[0] == item[0] and now-current[1] < 4.):
                    self.pending[source] = (item[0], item[1], current[1])
                    self.retry_at = now + .3
            except Exception as exc:
                self.get_logger().warning('阶段播报失败：' + type(exc).__name__)
        self.pending = OrderedDict((source, item) for source, item in self.pending.items() if now-item[2] < 4.)
        if self.pending and now >= self.retry_at and self.client.service_is_ready():
            source, item = self.pending.popitem(last=False)
            self.inflight_started = now
            self.inflight = (source, item, self.client.call_async(SayText.Request(text=item[1])))


def main(args=None):
    rclpy.init(args=args)
    node = PhaseAnnouncements()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
