"""Wake-triggered streaming speech recognition using iFLYTEK IAT."""

import json
import os
import queue
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int8, String
from std_srvs.srv import Trigger

from .audio import ArecordCapture, pcm_rms_s16le
from .auth import build_signed_websocket_url
from .protocol import IatResultAssembler, make_iat_frame


class XfyunAsrNode(Node):
    def __init__(self):
        super().__init__('xfyun_asr')
        self.declare_parameter('endpoint', 'wss://iat-api.xfyun.cn/v2/iat')
        self.declare_parameter('capture_device', 'default')
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('frame_ms', 40)
        self.declare_parameter('language', 'zh_cn')
        self.declare_parameter('accent', 'mandarin')
        self.declare_parameter('energy_threshold', 2800.0)
        self.declare_parameter('speech_onset_ms', 160)
        self.declare_parameter('silence_ms', 1200)
        self.declare_parameter('speech_timeout_sec', 5.0)
        self.declare_parameter('max_utterance_sec', 15.0)
        self.declare_parameter('response_timeout_sec', 5.0)
        self.declare_parameter('wake_topic', '/awake_flag')
        self.declare_parameter('wake_text_topic', '')
        self.declare_parameter('trusted_wake_text', '小车唤醒')
        self.declare_parameter('speaking_topic', '/voice/speaking')
        self.declare_parameter('output_topic', '/voice/asr_text')

        wake_topic = self.get_parameter('wake_topic').value
        wake_text_topic = self.get_parameter('wake_text_topic').value
        speaking_topic = self.get_parameter('speaking_topic').value
        output_topic = self.get_parameter('output_topic').value
        self._text_pub = self.create_publisher(String, output_topic, 10)
        self._state_pub = self.create_publisher(String, '/voice/asr_state', 10)
        self._wake_sub = None
        if wake_topic:
            self._wake_sub = self.create_subscription(
                Int8, wake_topic, self._on_wake, 10)
        self._wake_text_sub = None
        if wake_text_topic:
            self._wake_text_sub = self.create_subscription(
                String, wake_text_topic, self._on_wake_text, 10)
        self._speaking_sub = self.create_subscription(
            Bool, speaking_topic, self._on_speaking, 10)
        self._trigger_srv = self.create_service(
            Trigger, '/voice/start_listening', self._on_trigger)

        self._speaking = False
        self._busy = False
        self._lock = threading.Lock()
        self._requests = queue.Queue(maxsize=1)
        self._stop = threading.Event()
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()
        self._publish_state('IDLE')

    def _publish_state(self, value):
        message = String()
        message.data = value
        self._state_pub.publish(message)

    def _on_speaking(self, message):
        self._speaking = bool(message.data)

    def _on_wake(self, message):
        if int(message.data) == 1:
            self._enqueue_listen('wake_flag')

    def _on_wake_text(self, message):
        trusted_text = self.get_parameter('trusted_wake_text').value
        if message.data.strip() == trusted_text:
            self._enqueue_listen('trusted_wake')

    def _on_trigger(self, request, response):
        del request
        accepted, detail = self._enqueue_listen('manual')
        response.success = accepted
        response.message = detail
        return response

    def _enqueue_listen(self, source):
        with self._lock:
            if self._speaking:
                return False, 'TTS 正在播放，暂不开始识别'
            if self._busy or not self._requests.empty():
                return False, 'ASR 已在工作'
            self._requests.put_nowait(source)
            return True, '已开始一次语音识别'

    def _worker_loop(self):
        while not self._stop.is_set():
            try:
                item = self._requests.get(timeout=0.2)
            except queue.Empty:
                continue
            if item is None:
                break
            with self._lock:
                self._busy = True
            try:
                self._listen_once(item)
            except Exception as exc:  # hardware/network errors must not kill the node
                self.get_logger().error(str(exc))
                self._publish_state(f'ERROR: {exc}')
            finally:
                with self._lock:
                    self._busy = False
                if not self._stop.is_set():
                    self._publish_state('IDLE')
                self._requests.task_done()

    @staticmethod
    def _receive_one(ws, assembler, timeout):
        import websocket
        previous_timeout = ws.gettimeout()
        try:
            ws.settimeout(timeout)
            try:
                raw = ws.recv()
            except websocket.WebSocketTimeoutException:
                return False
        finally:
            # A 1 ms polling timeout must never leak into the next audio send.
            # On a real Wi-Fi link that turns harmless receive polling into
            # intermittent ``write operation timed out`` failures.
            ws.settimeout(previous_timeout)
        if not raw:
            return False
        if isinstance(raw, bytes):
            raw = raw.decode('utf-8')
        _, final = assembler.consume(json.loads(raw))
        return final

    def _listen_once(self, trigger_source):
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
        frame_ms = int(self.get_parameter('frame_ms').value)
        frame_bytes = sample_rate * 2 * frame_ms // 1000
        energy_threshold = float(self.get_parameter('energy_threshold').value)
        speech_onset_ms = int(self.get_parameter('speech_onset_ms').value)
        silence_limit = int(self.get_parameter('silence_ms').value)
        speech_timeout = float(self.get_parameter('speech_timeout_sec').value)
        max_utterance = float(self.get_parameter('max_utterance_sec').value)
        response_timeout = float(self.get_parameter('response_timeout_sec').value)
        language = self.get_parameter('language').value
        accent = self.get_parameter('accent').value
        device = self.get_parameter('capture_device').value

        url = build_signed_websocket_url(endpoint, api_key, api_secret)
        capture = ArecordCapture(device, sample_rate)
        ws = None
        assembler = IatResultAssembler()
        self._publish_state('CONNECTING')
        try:
            ws = websocket.create_connection(url, timeout=5, enable_multithread=True)
            capture.start()
            self._publish_state('LISTENING')
            started = time.monotonic()
            speech_started = False
            speech_candidate_ms = 0
            silence_elapsed = 0
            first_frame = True
            server_final = False

            while not self._stop.is_set() and not self._speaking:
                audio = capture.read(frame_bytes)
                rms = pcm_rms_s16le(audio)
                if rms >= energy_threshold:
                    speech_candidate_ms += frame_ms
                    if speech_candidate_ms >= speech_onset_ms:
                        speech_started = True
                    if speech_started:
                        silence_elapsed = 0
                elif speech_started:
                    silence_elapsed += frame_ms
                else:
                    speech_candidate_ms = 0

                status = 0 if first_frame else 1
                frame = make_iat_frame(
                    app_id, audio, status,
                    language=language,
                    accent=accent,
                    sample_rate=sample_rate,
                    vad_eos_ms=silence_limit,
                )
                ws.send(json.dumps(frame, ensure_ascii=False))
                first_frame = False
                server_final = self._receive_one(ws, assembler, 0.001)
                elapsed = time.monotonic() - started
                if server_final:
                    break
                if not speech_started and elapsed >= speech_timeout:
                    break
                if speech_started and silence_elapsed >= silence_limit:
                    break
                if elapsed >= max_utterance:
                    break

            if not server_final:
                ws.send(json.dumps(make_iat_frame(
                    app_id, b'', 2,
                    language=language,
                    accent=accent,
                    sample_rate=sample_rate,
                    vad_eos_ms=silence_limit,
                )))
                self._publish_state('RECOGNIZING')
                deadline = time.monotonic() + response_timeout
                while time.monotonic() < deadline:
                    remaining = max(0.01, min(0.5, deadline - time.monotonic()))
                    if self._receive_one(ws, assembler, remaining):
                        server_final = True
                        break

            text = assembler.text.strip()
            trusted_wake = trigger_source == 'trusted_wake'
            if not speech_started and not (trusted_wake and text):
                if text:
                    self.get_logger().warning(
                        f'本地 VAD 未检测到语音，丢弃云端文本: {text}')
                else:
                    self.get_logger().info('未检测到超过能量阈值的语音')
            elif text:
                message = String()
                message.data = text
                self._text_pub.publish(message)
                if trusted_wake and not speech_started:
                    self.get_logger().info('硬件唤醒已确认，接受低音量语音结果')
                self.get_logger().info(f'识别结果: {text}')
            elif not server_final:
                raise RuntimeError('讯飞 ASR 最终结果等待超时')
            else:
                self.get_logger().info('检测到语音，但讯飞未返回文本')
        finally:
            capture.close()
            if ws is not None:
                ws.close()

    def destroy_node(self):
        self._stop.set()
        try:
            self._requests.put_nowait(None)
        except queue.Full:
            pass
        self._worker.join(timeout=2)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = XfyunAsrNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
