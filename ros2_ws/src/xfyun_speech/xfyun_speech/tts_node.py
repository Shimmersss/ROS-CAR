"""Streaming iFLYTEK TTS with direct ALSA playback."""

import json
import os
import queue
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .audio import AplaySink
from .auth import build_signed_websocket_url
from .protocol import decode_tts_response, make_tts_request


class XfyunTtsNode(Node):
    def __init__(self):
        super().__init__('xfyun_tts')
        self.declare_parameter('endpoint', 'wss://tts-api.xfyun.cn/v2/tts')
        self.declare_parameter('input_topic', '/voice/tts_text')
        self.declare_parameter('playback_device', 'default')
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('voice_name', 'xiaoyan')
        self.declare_parameter('speed', 50)
        self.declare_parameter('volume', 50)
        self.declare_parameter('pitch', 50)
        self.declare_parameter('max_text_chars', 500)

        input_topic = self.get_parameter('input_topic').value
        self._subscription = self.create_subscription(
            String, input_topic, self._on_text, 10)
        self._state_pub = self.create_publisher(String, '/voice/tts_state', 10)
        self._speaking_pub = self.create_publisher(Bool, '/voice/speaking', 10)
        self._queue = queue.Queue(maxsize=5)
        self._stop = threading.Event()
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()
        self._publish_state('IDLE')

    def _publish_state(self, value):
        message = String()
        message.data = value
        self._state_pub.publish(message)

    def _publish_speaking(self, value):
        message = Bool()
        message.data = bool(value)
        self._speaking_pub.publish(message)

    def _on_text(self, message):
        text = message.data.strip()
        if not text:
            return
        limit = int(self.get_parameter('max_text_chars').value)
        if len(text) > limit:
            text = text[:limit] + '。'
            self.get_logger().warning(f'TTS 文本超过 {limit} 字，已截断')
        try:
            self._queue.put_nowait(text)
        except queue.Full:
            self.get_logger().warning('TTS 队列已满，丢弃新文本')

    def _worker_loop(self):
        while not self._stop.is_set():
            try:
                text = self._queue.get(timeout=0.2)
            except queue.Empty:
                continue
            if text is None:
                break
            try:
                self._speak(text)
            except Exception as exc:
                self.get_logger().error(str(exc))
                self._publish_state(f'ERROR: {exc}')
            finally:
                self._publish_speaking(False)
                if not self._stop.is_set():
                    self._publish_state('IDLE')
                self._queue.task_done()

    def _speak(self, text):
        try:
            import websocket
        except ImportError as exc:
            raise RuntimeError('缺少 python3-websocket/websocket-client') from exc

        app_id = os.environ.get('XFYUN_APP_ID', '').strip()
        api_key = os.environ.get('XFYUN_API_KEY', '').strip()
        api_secret = os.environ.get('XFYUN_API_SECRET', '').strip()
        if not all((app_id, api_key, api_secret)):
            raise RuntimeError('未设置 XFYUN_APP_ID/XFYUN_API_KEY/XFYUN_API_SECRET')

        endpoint = self.get_parameter('endpoint').value
        sample_rate = int(self.get_parameter('sample_rate').value)
        device = self.get_parameter('playback_device').value
        request = make_tts_request(
            app_id,
            text,
            voice_name=self.get_parameter('voice_name').value,
            sample_rate=sample_rate,
            speed=int(self.get_parameter('speed').value),
            volume=int(self.get_parameter('volume').value),
            pitch=int(self.get_parameter('pitch').value),
        )
        url = build_signed_websocket_url(endpoint, api_key, api_secret)
        sink = AplaySink(device, sample_rate)
        ws = None
        self._publish_speaking(True)
        self._publish_state('SPEAKING')
        try:
            ws = websocket.create_connection(url, timeout=8)
            sink.start()
            ws.send(json.dumps(request, ensure_ascii=False))
            complete = False
            while not complete and not self._stop.is_set():
                raw = ws.recv()
                if isinstance(raw, bytes):
                    raw = raw.decode('utf-8')
                audio, complete = decode_tts_response(json.loads(raw))
                sink.write(audio)
            sink.close()
            sink = None
        finally:
            if sink is not None:
                sink.close()
            if ws is not None:
                ws.close()

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
    node = XfyunTtsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
