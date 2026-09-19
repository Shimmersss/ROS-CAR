"""ROS 2 text bridge from ASR to DeepSeek and TTS."""

import os
import json
import queue
import re
import threading
import time
import uuid

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .acknowledgement import (
    AcknowledgementConfig,
    match_acknowledgement,
    match_wake_trigger,
)
from .client import DeepSeekClient
from .response_log import append_response


_THINK_BLOCK = re.compile(r'<think>.*?</think>', re.DOTALL)


class DeepSeekChatNode(Node):
    def __init__(self):
        super().__init__('deepseek_chat')
        self.declare_parameter('base_url', 'https://api.deepseek.com')
        self.declare_parameter('model', 'deepseek-v4-flash')
        self.declare_parameter('thinking_enabled', False)
        self.declare_parameter('timeout_sec', 30.0)
        self.declare_parameter('max_tokens', 300)
        self.declare_parameter('history_turns', 6)
        self.declare_parameter(
            'system_prompt',
            '你是运行在室内 ROS 2 小车上的中文语音助手。回答简洁、口语化，'
            '不要使用 Markdown。用户明确要求蜂鸣器响时必须调用 buzz 工具。'
            '当前尚未开放车辆运动控制，不要声称已经执行车辆动作。',
        )
        self.declare_parameter('input_topic', '/voice/asr_text')
        self.declare_parameter('answer_topic', '/voice/assistant_text')
        self.declare_parameter('tts_topic', '/voice/tts_text')
        self.declare_parameter('tool_call_topic', '/voice/tool_call')
        self.declare_parameter('response_log_path', '')
        self.declare_parameter('enable_tools', True)
        self.declare_parameter(
            'ignored_phrases',
            ['小车唤醒', '你好小微', '小微小微', '你好小薇', '小薇小薇'],
        )
        # Wake-word acknowledgements ("在") are answered locally: no API call,
        # no history entry, and the reply is not spoken into a running TTS.
        self.declare_parameter('ack_enabled', True)
        self.declare_parameter(
            'ack_phrases', ['在', '我在', '在这儿', '在这里'])
        self.declare_parameter('ack_reply', '我在')
        # Answer the wake word itself, before the user asks anything. Kept out of
        # the dialogue history and only spoken, never published as an answer.
        self.declare_parameter('wake_reply_enabled', True)
        self.declare_parameter('wake_reply_topic', '/voice_words')
        self.declare_parameter('wake_reply_trigger', '小车唤醒')
        self.declare_parameter('wake_reply_text', '我在')
        # Announces every accepted utterance so the ASR node can finalise a
        # short acknowledgement without waiting out its silence window.
        self.declare_parameter('talk_topic', '/voice/talk')

        self._answer_pub = self.create_publisher(
            String, self.get_parameter('answer_topic').value, 10)
        self._tts_pub = self.create_publisher(
            String, self.get_parameter('tts_topic').value, 10)
        self._tool_pub = self.create_publisher(
            String, self.get_parameter('tool_call_topic').value, 10)
        self._state_pub = self.create_publisher(String, '/voice/chat_state', 10)
        talk_topic = str(self.get_parameter('talk_topic').value)
        self._talk_pub = (
            self.create_publisher(String, talk_topic, 10) if talk_topic else None)

        self._ignored = set(self.get_parameter('ignored_phrases').value)
        self._ack = AcknowledgementConfig(
            phrases=tuple(self.get_parameter('ack_phrases').value),
            reply=str(self.get_parameter('ack_reply').value),
        )
        self._ack_enabled = bool(self.get_parameter('ack_enabled').value)
        self._system_prompt = self.get_parameter('system_prompt').value
        self._history = []
        self._queue = queue.Queue(maxsize=3)
        self._stop = threading.Event()
        self._tts_speaking = False
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()
        self._subscription = self.create_subscription(
            String, self.get_parameter('input_topic').value, self._on_text, 10)
        self._speaking_subscription = self.create_subscription(
            Bool, '/voice/speaking', self._on_speaking, 10)
        self._wake_reply_enabled = bool(
            self.get_parameter('wake_reply_enabled').value)
        self._wake_reply_text = str(
            self.get_parameter('wake_reply_text').value).strip()
        self._wake_reply_trigger = str(
            self.get_parameter('wake_reply_trigger').value).strip()
        wake_reply_topic = str(self.get_parameter('wake_reply_topic').value)
        self._wake_subscription = None
        if self._wake_reply_enabled and wake_reply_topic and self._wake_reply_text:
            self._wake_subscription = self.create_subscription(
                String, wake_reply_topic, self._on_wake, 10)
        self._publish_state('IDLE')

    def _publish_state(self, value):
        message = String()
        message.data = value
        self._state_pub.publish(message)

    def _on_text(self, message):
        text = message.data.strip()
        if not text or text in self._ignored:
            return
        # Announce first: the ASR node may still be finishing this utterance and
        # can cut its silence wait short. Text is forwarded verbatim; this
        # module makes no decision from it.
        if self._talk_pub is not None:
            talk = String()
            talk.data = text
            self._talk_pub.publish(talk)
        if self._ack_enabled:
            reply = match_acknowledgement(text, self._ack)
            if reply is not None:
                # Answer immediately instead of queueing: the queue may still
                # hold the previous question, and a stale "我在" is worse than
                # none. The acknowledgement stays out of the history as well.
                self._publish_reply(reply)
                self.get_logger().info(f'唤醒应答: {text} -> {reply}')
                return
        try:
            self._queue.put_nowait(text)
        except queue.Full:
            self.get_logger().warning('DeepSeek 请求队列已满，丢弃新问题')

    def _on_wake(self, message):
        """Speak a short greeting the moment the hardware wake word arrives.

        This is not a dialogue turn: it is never published on the answer topic
        and never enters the history, so the user's real question is still
        handled as if the greeting had not happened.
        """
        if not match_wake_trigger(message.data, self._wake_reply_trigger):
            return
        self._publish_reply(self._wake_reply_text, publish_answer=False)
        self.get_logger().info(f'唤醒应答: {self._wake_reply_text}')

    def _publish_reply(self, answer, publish_answer=True):
        message = String()
        message.data = answer
        if publish_answer:
            self._answer_pub.publish(message)
        if self._tts_speaking:
            self.get_logger().info('TTS 正在播放，跳过应答语音')
            return
        self._tts_pub.publish(message)

    def _on_speaking(self, message):
        self._tts_speaking = bool(message.data)

    def _messages_for(self, user_text):
        return [
            {'role': 'system', 'content': self._system_prompt},
            *self._history,
            {'role': 'user', 'content': user_text},
        ]

    def _remember(self, user_text, answer):
        self._history.extend([
            {'role': 'user', 'content': user_text},
            {'role': 'assistant', 'content': answer},
        ])
        history_turns = max(0, int(self.get_parameter('history_turns').value))
        self._history = self._history[-2 * history_turns:] if history_turns else []

    def _client(self):
        api_key = os.environ.get('DEEPSEEK_API_KEY', '').strip()
        if not api_key:
            raise RuntimeError('未设置 DEEPSEEK_API_KEY')
        return DeepSeekClient(
            api_key,
            base_url=self.get_parameter('base_url').value,
            model=self.get_parameter('model').value,
            timeout_sec=float(self.get_parameter('timeout_sec').value),
            max_tokens=int(self.get_parameter('max_tokens').value),
            thinking_enabled=bool(self.get_parameter('thinking_enabled').value),
        )

    @staticmethod
    def _tools():
        return [{
            'type': 'function',
            'function': {
                'name': 'buzz',
                'description': '让机器人上的蜂鸣器短促鸣响一次。只有用户明确要求蜂鸣器响、鸣叫或蜂鸣时才调用。',
                'parameters': {
                    'type': 'object',
                    'properties': {
                        'duration_ms': {
                            'type': 'integer',
                            'minimum': 100,
                            'maximum': 2000,
                            'description': '鸣响持续时间，默认300毫秒。',
                        },
                    },
                    'additionalProperties': False,
                },
            },
        }]

    def _handle_tool_calls(self, message):
        tool_calls = message.get('tool_calls') or []
        if len(tool_calls) != 1:
            raise RuntimeError('只允许一次调用一个机器人工具')
        function = tool_calls[0].get('function') or {}
        if function.get('name') != 'buzz':
            raise RuntimeError(f"拒绝未知工具: {function.get('name')}")
        try:
            arguments = json.loads(function.get('arguments') or '{}')
        except json.JSONDecodeError as exc:
            raise RuntimeError('蜂鸣器工具参数不是有效 JSON') from exc
        if set(arguments) - {'duration_ms'}:
            raise RuntimeError('蜂鸣器工具包含未知参数')
        duration_ms = arguments.get('duration_ms', 300)
        if isinstance(duration_ms, bool) or not isinstance(duration_ms, int):
            raise RuntimeError('duration_ms 必须是整数')
        if not 100 <= duration_ms <= 2000:
            raise RuntimeError('duration_ms 必须在 100 到 2000 之间')
        command = {
            'request_id': str(uuid.uuid4()),
            'name': 'buzz',
            'arguments': {'duration_ms': duration_ms},
        }
        output = String()
        output.data = json.dumps(command, ensure_ascii=False)
        self._tool_pub.publish(output)
        return f'已请求蜂鸣器鸣响{duration_ms}毫秒。'

    def _worker_loop(self):
        while not self._stop.is_set():
            try:
                user_text = self._queue.get(timeout=0.2)
            except queue.Empty:
                continue
            if user_text is None:
                break
            try:
                self._publish_state('THINKING')
                client = self._client()
                last_error = None
                for attempt in range(2):
                    try:
                        tools = self._tools() if bool(
                            self.get_parameter('enable_tools').value) else None
                        message = client.complete(
                            self._messages_for(user_text), tools=tools, tool_choice='auto')
                        break
                    except Exception as exc:
                        last_error = exc
                        if attempt == 0:
                            time.sleep(0.5)
                else:
                    raise last_error
                if message.get('tool_calls'):
                    answer = self._handle_tool_calls(message)
                else:
                    answer = message.get('content') or ''
                answer = _THINK_BLOCK.sub('', answer).strip()
                if not answer:
                    raise RuntimeError('移除思考标签后回答为空')
                self._remember(user_text, answer)
                response_log_path = str(
                    self.get_parameter('response_log_path').value).strip()
                if response_log_path:
                    try:
                        append_response(response_log_path, answer)
                    except OSError as exc:
                        self.get_logger().error(f'写入 DeepSeek 回答文件失败: {exc}')
                message = String()
                message.data = answer
                self._answer_pub.publish(message)
                self._tts_pub.publish(message)
                self.get_logger().info(f'回答: {answer}')
            except Exception as exc:
                self.get_logger().error(str(exc))
                self._publish_state(f'ERROR: {exc}')
            finally:
                if not self._stop.is_set():
                    self._publish_state('IDLE')
                self._queue.task_done()

    def destroy_node(self):
        self._stop.set()
        try:
            self._queue.put_nowait(None)
        except queue.Full:
            pass
        self._worker.join(timeout=2)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DeepSeekChatNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
