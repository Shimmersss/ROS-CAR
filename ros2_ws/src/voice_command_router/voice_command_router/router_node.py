"""Fixed intent router; global authorization belongs only to motion_guard."""
import json
import math
import os
import pathlib
import sys
import time
import uuid
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from std_msgs.msg import String, Float32, Bool
from std_srvs.srv import Trigger

_ws = pathlib.Path(os.environ.get('ROSCAR_WS', '/home/wheeltec/ROSCAR-current/ros2_ws'))
_iface = _ws.parent / 'install/roscar_interfaces/local/lib/python3.10/dist-packages'
if _iface.is_dir():
    sys.path.insert(0, str(_iface))
from person_interfaces.msg import TargetState
from roscar_interfaces.msg import VoiceCommand, VoiceCommandResult, MotionRequest
from roscar_interfaces.srv import SetControlMode, AuthorizeMotion
from .validation import CommandValidationError, parse_intent, validate_mode
from .session import VoiceSession


class VoiceCommandRouter(Node):
    def __init__(self, **kwargs):
        super().__init__('voice_command_router', **kwargs)
        for name, value in dict(enabled=True, local_parser_enabled=True,
                                input_topic='/voice/asr_text', command_topic='/voice/command',
                                result_topic='/voice/command_result', followup_window_s=8.0,
                                continuous_wake_text='小车唤醒').items():
            self.declare_parameter(name, value)
        get = lambda key: self.get_parameter(key).value
        self.session = VoiceSession(float(get('followup_window_s')))
        self._result_pub = self.create_publisher(VoiceCommandResult, get('result_topic'), 10)
        self._legacy_result_pub = self.create_publisher(String, '/voice/tool_result', 10)
        self._answer_pub = self.create_publisher(String, '/voice/assistant_text', 10)
        self._tts_pub = self.create_publisher(String, '/voice/tts_text', 10)
        self._motion_pub = self.create_publisher(MotionRequest, '/voice/motion_request', 10)
        self._set_mode = self.create_client(SetControlMode, '/control/set_mode')
        self._arm = self.create_client(AuthorizeMotion, '/control/authorize')
        self._control_services = {action: self.create_client(Trigger, topic) for action, topic in {
            'STOP': '/control/stop', 'DISARM': '/control/disarm', 'ESTOP': '/control/estop',
            'RESET_ESTOP': '/control/reset_estop', 'RESET_FAULT': '/control/reset_fault'}.items()}
        self._listen_client = self.create_client(Trigger, '/voice/start_listening')
        self._cancel_listen = self.create_client(Trigger, '/voice/cancel_listening')
        self._cancel_tts = self.create_client(Trigger, '/voice/cancel_speech')
        self._state, self._state_at = {}, -math.inf
        self._seen, self._generation, self._pending_arm = {}, 0, None
        self._arm_inflight = False
        self._power = self._target = None
        self._next_listen_at = self._last_listen_attempt = 0.
        self._hold_at, self._tts_hold, self._speech_hold = 0., False, False
        self.create_subscription(String, get('input_topic'), self._on_asr, 10)
        self.create_subscription(String, '/voice_words', self._on_wake, 10)
        self.create_subscription(String, '/voice/asr_state', self._on_asr_state, 10)
        self.create_subscription(Bool, '/voice/speaking', self._on_speaking, 10)
        self.create_subscription(VoiceCommand, get('command_topic'), self._on_command, 10)
        self.create_subscription(String, '/control/state', self._on_state, 10)
        self.create_subscription(Float32, '/PowerVoltage', lambda msg: setattr(self, '_power', msg.data), 10)
        self.create_subscription(TargetState, '/perception/target_state', lambda msg: setattr(self, '_target', msg), 10)
        self.create_timer(.05, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _new_command(self, action, **kwargs):
        result = VoiceCommand()
        result.stamp = self.get_clock().now().to_msg()
        result.request_id = kwargs.get('request_id', 'local-' + str(uuid.uuid4()))
        result.action, result.mode, result.query = action, kwargs.get('mode', ''), kwargs.get('query', '')
        return result

    def _on_wake(self, message):
        if message.data.strip() != self.get_parameter('continuous_wake_text').value:
            return
        self._cancel_pending_authorization()
        self.session.wake(time.monotonic())
        self._tts_hold = self._speech_hold = False
        self._generation += 1
        self._pending_arm = None
        for client in (self._cancel_listen, self._cancel_tts):
            if client.service_is_ready():
                client.call_async(Trigger.Request())
        self._speak('已唤醒，请说指令')
        self._request_next_listen()

    def _on_speaking(self, message):
        now = time.monotonic()
        if message.data and self.session.active(now):
            self._tts_hold, self._hold_at = True, now
        elif not message.data and self._tts_hold:
            self._tts_hold = False
            self.session.success(now)

    def _on_asr_state(self, message):
        now = time.monotonic()
        if message.data in ('SPEECH', 'RECOGNIZING') and self.session.active(now):
            self._speech_hold, self._hold_at = True, now
        elif message.data == 'IDLE':
            self._speech_hold = False
            self._request_next_listen()

    def _session_active(self):
        now = time.monotonic()
        return self.session.active(now) or ((self._tts_hold or self._speech_hold) and now - self._hold_at < 25)

    def _request_next_listen(self):
        if self._session_active():
            self._next_listen_at = time.monotonic() + .35

    def _cancel_pending_authorization(self):
        if (self._pending_arm or self._arm_inflight) and self._control_services['STOP'].service_is_ready():
            # Revoke the server-side epoch too: ignoring a late future alone
            # cannot undo an authorization request already in transport.
            self._control_services['STOP'].call_async(Trigger.Request())
        self._generation += 1
        self._pending_arm, self._arm_inflight = None, False

    def _close_session(self):
        self._cancel_pending_authorization()
        self.session.close()
        self._tts_hold = self._speech_hold = False
        self._next_listen_at = 0.
        if self._cancel_listen.service_is_ready():
            self._cancel_listen.call_async(Trigger.Request())

    def _on_asr(self, message):
        try:
            parsed = parse_intent(message.data)
        except CommandValidationError as exc:
            if self._session_active():
                self._speak(f'无法执行：{exc}')
                self._request_next_listen()
            return
        if parsed and parsed['action'] in ('STOP', 'ESTOP', 'DISARM'):
            self._on_command(self._new_command(**parsed))
            return
        if not self._session_active():
            return  # Never execute a late result after session expiry/cancel.
        if not self.get_parameter('local_parser_enabled').value:
            self._speak('语音控制规则已关闭')
        elif parsed is None:
            self._speak('未识别到单一控制指令，请重新说明')
        elif parsed['action'] == 'END_SESSION':
            self._close_session()
            self._speak('已退出语音会话')
            return
        elif parsed['action'] in ('MOVE', 'TURN'):
            if not self.get_parameter('enabled').value:
                self._speak('语音控制已关闭')
                return
            request = MotionRequest()
            request.stamp = self.get_clock().now().to_msg()
            request.request_id = 'local-' + str(uuid.uuid4())
            request.authority_epoch = int(self._state.get('authority_epoch', 0))
            request.action, request.direction, request.style = parsed['action'], parsed['direction'], parsed['style']
            request.speed_specified = parsed['speed'] is not None
            request.speed = parsed['speed'] if request.speed_specified else 0.
            request.duration_specified = parsed['duration_s'] is not None
            request.duration_s = parsed['duration_s'] if request.duration_specified else 0.
            self._motion_pub.publish(request)
            self.session.success(time.monotonic())
        else:
            self._on_command(self._new_command(**parsed))
        self._request_next_listen()

    def _on_command(self, command):
        try:
            action = command.action
            if action in ('STOP', 'DISARM', 'ESTOP'):
                self._generation += 1
                self._pending_arm = None
                self._call_service(self._control_services[action], command)
                return
            if not self.get_parameter('enabled').value:
                raise CommandValidationError('语音控制已关闭')
            stamp = command.stamp.sec + command.stamp.nanosec * 1e-9
            age = self.get_clock().now().nanoseconds * 1e-9 - stamp
            if stamp <= 0 or not 0 <= age <= 2:
                raise CommandValidationError('命令已过期或时间戳无效')
            now = time.monotonic()
            self._seen = {key: at for key, at in self._seen.items() if now - at < 60}
            if not command.request_id or command.request_id in self._seen:
                raise CommandValidationError('命令标识无效或重复')
            self._seen[command.request_id] = now
            if action in ('ARM', 'SET_MODE'):
                self._set_control_mode('EXTERNAL' if action == 'ARM' else validate_mode(command.mode), command)
            elif action in ('RESET_ESTOP', 'RESET_FAULT'):
                self._call_service(self._control_services[action], command)
            elif action == 'QUERY_STATUS':
                self._query_status(command)
            else:
                raise CommandValidationError(f'未知语音动作: {action}；运动请使用 MotionRequest')
        except (ValueError, TypeError) as exc:
            self._publish_result(command, False, str(exc))

    def _set_control_mode(self, mode, command):
        self._generation += 1
        self._pending_arm = None
        if mode == 'IDLE':
            self._call_service(self._control_services['STOP'], command)
            return
        if not self._set_mode.service_is_ready():
            self._publish_result(command, False, '控制模式服务不可用')
            return
        request = SetControlMode.Request()
        request.mode = mode
        generation = self._generation
        future = self._set_mode.call_async(request)
        def done(f):
            if generation != self._generation:
                return
            try:
                response = f.result()
                if response.success:
                    self._pending_arm = (command, mode, generation, response.authority_epoch, time.monotonic() + 3)
                else:
                    self._publish_result(command, False, response.message)
            except Exception as exc:
                self._publish_result(command, False, str(exc))
        future.add_done_callback(done)

    def _call_service(self, client, command):
        if not client.service_is_ready():
            self._publish_result(command, False, '控制服务不可用')
            return
        future = client.call_async(Trigger.Request())
        def done(f):
            try:
                result = f.result()
                self._publish_result(command, result.success, result.message)
            except Exception as exc:
                self._publish_result(command, False, str(exc))
        future.add_done_callback(done)

    def _on_state(self, message):
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict):
                raise ValueError('Invalid control state')
            self._state, self._state_at = value, time.monotonic()
        except (ValueError, TypeError):
            self._state, self._state_at = {}, -math.inf

    def _tick(self):
        now = time.monotonic()
        if not self._session_active():
            if self._next_listen_at:
                self._close_session()
        elif not self._tts_hold and (now >= self._next_listen_at or now - self._last_listen_attempt >= 1):
            if self._listen_client.service_is_ready():
                self._last_listen_attempt, self._next_listen_at = now, now + .35
                self._listen_client.call_async(Trigger.Request())
        if self._pending_arm:
            command, mode, generation, epoch, deadline = self._pending_arm
            if now >= deadline:
                self._pending_arm = None
                self._publish_result(command, False, '授权等待超时')
            elif (now - self._state_at < .3 and self._state.get('command_mode') == mode
                  and self._state.get('authority_epoch') == epoch and self._state.get('ready')
                  and self._arm.service_is_ready()):
                self._pending_arm = None
                request = AuthorizeMotion.Request()
                request.mode, request.authority_epoch = mode, epoch
                self._arm_inflight = True
                future = self._arm.call_async(request)
                def done(f):
                    if generation != self._generation:
                        return
                    self._arm_inflight = False
                    try:
                        result = f.result()
                        self._publish_result(command, result.success, result.message)
                    except Exception as exc:
                        self._publish_result(command, False, str(exc))
                future.add_done_callback(done)

    def _query_status(self, command):
        if command.query == 'unsupported_navigation':
            self._publish_result(command, False, '当前未接入导航目标和地点映射')
            return
        if time.monotonic() - self._state_at >= .3:
            self._publish_result(command, False, '当前控制状态不可用')
            return
        modes = {'IDLE': '空闲', 'EXTERNAL': '手动', 'FOLLOW': '跟随'}
        authority = {'DISARMED': '未授权', 'ARMED_MANUAL': '手动已授权', 'ARMED_FOLLOW': '跟随已授权',
                     'ESTOP': '紧急制动已锁存', 'FAULT': '故障待复位'}
        parts = [f"当前{modes.get(self._state.get('command_mode'), '未知')}模式",
                 authority.get(self._state.get('authority_state'), '授权状态未知')]
        if self._power is not None and math.isfinite(float(self._power)):
            parts.append(f'电压 {self._power:.2f} V')
        if self._target is not None:
            parts.append('已锁定目标' if self._target.status == TargetState.TRACKING else '没有有效跟随目标')
        self._publish_result(command, True, '，'.join(parts))

    def _publish_result(self, command, success, message):
        result = VoiceCommandResult()
        result.stamp, result.request_id, result.action = self.get_clock().now().to_msg(), command.request_id, command.action
        result.success, result.message = bool(success), str(message)
        result.command_mode, result.guard_mode = self._state.get('command_mode', ''), self._state.get('mode', '')
        self._result_pub.publish(result)
        self._legacy_result_pub.publish(String(data=json.dumps({'request_id': result.request_id,
            'success': result.success, 'detail': result.message}, ensure_ascii=False)))
        if success and self._session_active():
            self.session.success(time.monotonic())
        if command.request_id.startswith(('local-', 'legacy-')):
            feedback = result.message
            if success:
                feedback = {'STOP': '已停车，控制授权已撤销', 'DISARM': '控制授权已撤销',
                            'ESTOP': '紧急制动已锁存，需明确复位后重新授权',
                            'RESET_ESTOP': '紧急制动锁存已解除，仍需重新授权',
                            'RESET_FAULT': '故障已复位，仍需重新授权',
                            'ARM': '手动控制已授权'}.get(command.action, feedback)
                if command.action == 'SET_MODE':
                    feedback = '跟随控制已授权' if command.mode == 'FOLLOW' else '手动控制已授权'
            self._speak(feedback)

    def _speak(self, text):
        # Ordinary feedback templates avoid speaking the four KWS stop phrases.
        # KWS remains active during arbitrary external TTS; AEC needs real testing.
        text = str(text).replace('停止', '停车').replace('停下', '停车').replace('急停', '紧急制动').replace('别动', '保持静止')
        message = String(data=text)
        self._answer_pub.publish(message)
        self._tts_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandRouter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._close_session()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
