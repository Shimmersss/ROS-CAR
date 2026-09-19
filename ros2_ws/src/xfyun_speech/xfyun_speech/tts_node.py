"""Streaming iFLYTEK TTS with direct ALSA playback."""

import json
import os
import queue
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .audio import AplaySink, play_audio
from .auth import build_signed_websocket_url
from .protocol import decode_tts_response, make_tts_request
from .tts_cache import cache_dir, clip_name, wav_bytes


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
        self.declare_parameter('reply_cache', '~/.cache/roscar/tts')
        # Non-empty default: an empty list would be inferred as BYTE_ARRAY.
        self.declare_parameter('precache_texts', [''])
        self.declare_parameter('timing_log', True)

        input_topic = self.get_parameter('input_topic').value
        self._subscription = self.create_subscription(
            String, input_topic, self._on_text, 10)
        self._state_pub = self.create_publisher(String, '/voice/tts_state', 10)
        self._speaking_pub = self.create_publisher(Bool, '/voice/speaking', 10)
        self._queue = queue.Queue(maxsize=5)
        self._stop = threading.Event()
        self._cache_root = str(self.get_parameter('reply_cache').value)
        self._precache = self.pre_cached_clips()
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()
        self._publish_state('IDLE')

    def cache_arguments(self):
        """Everything that changes the rendered audio, for cache fingerprinting."""
        return (
            str(self.get_parameter('voice_name').value),
            int(self.get_parameter('speed').value),
            int(self.get_parameter('pitch').value),
            int(self.get_parameter('volume').value),
            int(self.get_parameter('sample_rate').value),
        )

    def cache_path(self, text):
        voice, speed, pitch, volume, rate = self.cache_arguments()
        return cache_dir(
            self._cache_root, voice, speed, pitch, volume, rate
        ) / clip_name(text, voice, speed, pitch, volume, rate)

    def pre_cached_clips(self):
        """Map pre-synthesised replies to their WAV path, synthesising missing ones.

        failure is never fatal: without a clip the reply falls back to the live
        WebSocket path, which is exactly the previous behaviour.
        """
        sample_rate = self.cache_arguments()[-1]
        clips = {}
        for raw in self.get_parameter('precache_texts').value or []:
            text = str(raw).strip()
            if not text:
                continue
            path = self.cache_path(text)
            if not path.exists():
                try:
                    path.parent.mkdir(parents=True, exist_ok=True)
                    pcm = self._synthesize_pcm(text)
                    path.write_bytes(wav_bytes(pcm, sample_rate))
                    self.get_logger().info(
                        f'预合成固定话术 {text!r}（{len(pcm) / 2 / sample_rate:.2f} 秒）')
                except Exception as exc:
                    self.get_logger().warning(f'预合成 {text!r} 失败，将改用在线合成: {exc}')
                    continue
            clips[text] = str(path)
        return clips

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
        requested_at = time.monotonic()
        device = self.get_parameter('playback_device').value
        cached = self._precache.get(text)
        self._publish_speaking(True)
        self._publish_state('SPEAKING')
        if cached:
            # A fixed reply plays straight from disk: no handshake, no round trip.
            play_audio(cached, device)
            self._timing('缓存播放=%.3f' % (time.monotonic() - requested_at))
            return
        self._stream_speech(text, device, requested_at)

    def _timing(self, message):
        """Latency evidence, one short line per utterance; off by parameter."""
        if bool(self.get_parameter('timing_log').value):
            self.get_logger().info('计时 ' + message)

    def _stream_speech(self, text, device, requested_at):
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
        try:
            connected_at = time.monotonic()
            ws = websocket.create_connection(url, timeout=8, enable_multithread=True)
            sink.start()
            ws.send(json.dumps(request, ensure_ascii=False))
            complete = False
            first_audio_at = None
            while not complete and not self._stop.is_set():
                raw = ws.recv()
                if isinstance(raw, bytes):
                    raw = raw.decode('utf-8')
                audio, complete = decode_tts_response(json.loads(raw))
                if audio and first_audio_at is None:
                    first_audio_at = time.monotonic()
                sink.write(audio)
            # Latency evidence: one WebSocket round trip plus iFLYTEK's first
            # audio chunk, which is what a pre-synthesised reply would remove.
            self._timing(
                'TTS连接=%.3f 首音频=%.3f'
                % (connected_at - requested_at,
                   (first_audio_at or time.monotonic()) - requested_at))
            sink.close()
            sink = None
        finally:
            if sink is not None:
                sink.close()
            if ws is not None:
                ws.close()

    def _synthesize_pcm(self, text):
        """Fetch the complete PCM for `text` without playing it."""
        import websocket

        app_id = os.environ.get('XFYUN_APP_ID', '').strip()
        api_key = os.environ.get('XFYUN_API_KEY', '').strip()
        api_secret = os.environ.get('XFYUN_API_SECRET', '').strip()
        if not all((app_id, api_key, api_secret)):
            raise RuntimeError('未设置 XFYUN_APP_ID/XFYUN_API_KEY/XFYUN_API_SECRET')
        sample_rate = int(self.get_parameter('sample_rate').value)
        request = make_tts_request(
            app_id,
            text,
            voice_name=self.get_parameter('voice_name').value,
            sample_rate=sample_rate,
            speed=int(self.get_parameter('speed').value),
            volume=int(self.get_parameter('volume').value),
            pitch=int(self.get_parameter('pitch').value),
        )
        url = build_signed_websocket_url(
            self.get_parameter('endpoint').value, api_key, api_secret)
        ws = websocket.create_connection(url, timeout=8, enable_multithread=True)
        try:
            ws.send(json.dumps(request, ensure_ascii=False))
            pcm = bytearray()
            complete = False
            while not complete:
                raw = ws.recv()
                if isinstance(raw, bytes):
                    raw = raw.decode('utf-8')
                audio, complete = decode_tts_response(json.loads(raw))
                pcm.extend(audio or b'')
            return bytes(pcm)
        finally:
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
