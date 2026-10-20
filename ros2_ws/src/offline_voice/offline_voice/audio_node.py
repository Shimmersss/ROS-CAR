"""The sole ALSA capture owner; always streams audio, including during TTS."""
import threading
import rclpy
from rclpy.node import Node
from roscar_interfaces.msg import AudioFrame
from xfyun_speech.audio import ArecordCapture
from .capture_loop import CaptureLoop


class AudioCaptureNode(Node):
    def __init__(self):
        super().__init__('voice_audio_capture')
        self.declare_parameter('capture_device', 'plughw:CARD=XFMDPV0018,DEV=0')
        self.pub = self.create_publisher(AudioFrame, '/voice/audio', 20)
        self.capture = ArecordCapture(self.get_parameter('capture_device').value, 16000,
                                      read_timeout_s=2.)
        self.loop = CaptureLoop(self.capture, self.publish_frame, self.capture_error)
        self.thread = threading.Thread(target=self.loop.run, daemon=True)
        self.thread.start()

    def publish_frame(self, sequence, pcm):
        message = AudioFrame()
        message.stamp = self.get_clock().now().to_msg()
        message.sequence, message.sample_rate, message.channels = sequence, 16000, 1
        message.pcm_s16le = list(pcm)
        self.pub.publish(message)

    def capture_error(self, error, retry_s):
        self.get_logger().error(
            f'Audio capture failed: {error}; retry in {retry_s:g}s; health remains expired until real audio resumes')

    def destroy_node(self):
        self.loop.stop()
        self.thread.join(timeout=3)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AudioCaptureNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
