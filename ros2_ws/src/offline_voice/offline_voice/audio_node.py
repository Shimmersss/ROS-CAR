"""The sole ALSA capture owner; always streams audio, including during TTS."""
import threading
import rclpy
from rclpy.node import Node
from roscar_interfaces.msg import AudioFrame
from xfyun_speech.audio import ArecordCapture


class AudioCaptureNode(Node):
    def __init__(self):
        super().__init__('voice_audio_capture')
        self.declare_parameter('capture_device', 'plughw:CARD=XFMDPV0018,DEV=0')
        self.pub = self.create_publisher(AudioFrame, '/voice/audio', 20)
        self.stop = threading.Event()
        self.capture = ArecordCapture(self.get_parameter('capture_device').value, 16000)
        self.thread = threading.Thread(target=self.run_capture, daemon=True)
        self.thread.start()

    def run_capture(self):
        try:
            self.capture.start()
            sequence = 0
            while not self.stop.is_set():
                pcm = self.capture.read(1280)
                if len(pcm) != 1280:
                    raise RuntimeError('Short audio frame')
                message = AudioFrame()
                message.stamp = self.get_clock().now().to_msg()
                message.sequence, message.sample_rate, message.channels = sequence, 16000, 1
                message.pcm_s16le = list(pcm)
                self.pub.publish(message)
                sequence += 1
        except Exception as exc:
            self.get_logger().error(f'Audio capture failed: {exc}; emergency health will expire')
        finally:
            self.capture.close()

    def destroy_node(self):
        self.stop.set()
        self.capture.close()
        self.thread.join(timeout=1)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AudioCaptureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
