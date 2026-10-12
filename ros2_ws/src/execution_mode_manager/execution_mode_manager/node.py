"""Generic, serial mode ownership. A RunMode goal lives until the mode exits."""
import json
import time
import uuid
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from std_msgs.msg import String
from execution_mode_interfaces.action import RunMode
from execution_mode_interfaces.msg import ModeCommand
from .catalog import Catalog


class ModeManager(Node):
    def __init__(self):
        super().__init__('execution_mode_manager')
        self.declare_parameter('catalog', '')
        self.catalog = Catalog.load(self.get_parameter('catalog').value)
        self.mode_clients = {name: ActionClient(self, RunMode, row['action'])
                        for name, row in self.catalog.modes.items()}
        self.state_pub = self.create_publisher(String, '/execution_mode/state', 10)
        self.control_pub = self.create_publisher(ModeCommand, '/execution_mode/control', 10)
        self.heartbeat_pub = self.create_publisher(String, '/execution_mode/heartbeat', 10)
        self.tts_pub = self.create_publisher(String, '/voice/tts_text', 10)
        self.create_subscription(ModeCommand, '/execution_mode/request', self.command, 10)
        self.active = None
        self.pending = None
        self.seen = {}
        self.timer = self.create_timer(.25, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def speak(self, text):
        # The independent emergency recognizer also hears local playback.
        text = str(text).replace('停止', '停车').replace('停下', '停车').replace('急停', '紧急制动').replace('别动', '保持静止')
        self.tts_pub.publish(String(data=text))

    def command(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        age = self.get_clock().now().nanoseconds*1e-9-stamp
        now = time.monotonic()
        self.seen = {k:v for k,v in self.seen.items() if now-v < 60.}
        if not msg.request_id or msg.request_id in self.seen or stamp <= 0 or not 0 <= age <= 2.:
            return
        self.seen[msg.request_id] = now
        if msg.operation == 'ENTER':
            if msg.mode_id not in self.mode_clients:
                self.speak('这个执行模式还没有配置。'); return
            if self.active:
                if self.active['mode_id'] == msg.mode_id and not self.active['exiting']:
                    self.speak('已经在这个模式中，可以重新拍摄或退出模式。'); return
                self.pending = msg
                self.exit_active()
            else:
                self.start(msg)
        elif msg.operation == 'EXIT':
            # A mode-specific negation must not cancel another mode.
            if self.active and (not msg.mode_id or msg.mode_id == self.active['mode_id']):
                self.pending = None
                self.exit_active()
            elif not self.active:
                self.speak('当前没有正在执行的模式。')
        else:
            if not self.active or self.active['exiting'] or (msg.mode_id and msg.mode_id != self.active['mode_id']):
                self.speak('请先进入对应的执行模式。'); return
            if msg.operation not in self.catalog.operations(self.active['mode_id']):
                self.speak('这个模式不支持这条操作。'); return
            self.forward(msg.operation, msg.request_id)

    def start(self, msg):
        client = self.mode_clients[msg.mode_id]
        if not client.server_is_ready():
            self.speak('这个模式的服务还没有就绪。'); return
        session = uuid.uuid4().hex
        self.active = dict(mode_id=msg.mode_id, session_id=session, phase='STARTING',
                           detail='', result_json='', handle=None, exiting=False,
                           began=time.monotonic(), last_feedback=time.monotonic(), cancel_at=0.)
        goal = RunMode.Goal(mode_id=msg.mode_id, session_id=session, request_id=msg.request_id)
        future = client.send_goal_async(goal, feedback_callback=lambda m: self.feedback(session, m.feedback))
        future.add_done_callback(lambda f: self.accepted(session, f))

    def accepted(self, session, future):
        try:
            handle = future.result()
            if not self.active or self.active['session_id'] != session:
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            if not handle.accepted:
                self.finished(session, None, '模式正在忙，未接受新任务。'); return
            self.active['handle'] = handle
            handle.get_result_async().add_done_callback(lambda f: self.finished(session, f))
            if self.active['exiting']:
                self.exit_active()
        except Exception as exc:
            self.finished(session, None, '模式启动失败：' + type(exc).__name__)

    def feedback(self, session, feedback):
        if self.active and self.active['session_id'] == session:
            self.active['last_feedback'] = time.monotonic()
            if not self.active['exiting']:
                self.active.update(phase=feedback.phase, detail=feedback.detail,
                                   result_json=feedback.result_json)

    def forward(self, operation, request_id=None):
        if not self.active:
            return
        msg = ModeCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.request_id = request_id or uuid.uuid4().hex
        msg.session_id = self.active['session_id']
        msg.mode_id = self.active['mode_id']
        msg.operation = operation
        self.control_pub.publish(msg)

    def exit_active(self):
        if not self.active:
            return
        self.active.update(exiting=True, phase='EXITING')
        self.forward('EXIT')
        handle = self.active['handle']
        if handle:
            handle.cancel_goal_async()
        self.active['cancel_at'] = time.monotonic()

    def finished(self, session, future, error=''):
        if not self.active or self.active['session_id'] != session:
            return
        was_exiting = self.active['exiting']
        if future:
            try:
                result = future.result().result
                if result.status not in ('exited', 'cancelled'):
                    error = '执行模式已停止：' + result.detail
            except Exception as exc:
                error = '读取模式结果失败：' + type(exc).__name__
        self.active = None
        if error:
            self.speak(error[:200])
        elif was_exiting and not self.pending:
            self.speak('已退出执行模式，小车保持停止。')
        pending, self.pending = self.pending, None
        if pending:
            self.start(pending)

    def tick(self):
        if self.active:
            now = time.monotonic()
            self.heartbeat_pub.publish(String(data=json.dumps({k:self.active[k] for k in ('mode_id', 'session_id')})))
            if not self.active['exiting'] and now-self.active['last_feedback'] > 6.:
                self.active['detail'] = '模式反馈超时，等待退出确认'
                self.exit_active()
            elif self.active['exiting'] and now-self.active['cancel_at'] > 1.:
                # Do not start another mode before the old action's final result.
                self.exit_active()
            state = {k:self.active[k] for k in ('mode_id', 'session_id', 'phase', 'detail', 'result_json')}
            state.update(active=True, busy=self.active['phase'] != 'READY')
        else:
            state = dict(mode_id='', session_id='', phase='IDLE', active=False, busy=False)
        self.state_pub.publish(String(data=json.dumps(state, ensure_ascii=False)))


def main(args=None):
    rclpy.init(args=args)
    node = ModeManager()
    try:
        rclpy.spin(node)
    finally:
        # Mode-side heartbeat watchdog performs cleanup even if the manager dies.
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
