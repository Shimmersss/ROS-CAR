"""Optional Jetson.GPIO buzzer adapter; disabled until a real pin is configured."""

import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String


class BuzzerGpioNode(Node):
    def __init__(self):
        super().__init__('buzzer_gpio')
        self.declare_parameter('enabled', False)
        self.declare_parameter('pin_numbering', 'BOARD')
        self.declare_parameter('pin', -1)
        self.declare_parameter('active_high', True)
        self.declare_parameter('input_topic', '/hardware/buzzer_duration_ms')
        self._state_pub = self.create_publisher(String, '/hardware/buzzer_state', 10)
        self._subscription = self.create_subscription(
            Int32, self.get_parameter('input_topic').value, self._on_command, 10)
        self._lock = threading.Lock()
        self._gpio = None
        self._configured_pin = None
        self._publish_state('DISABLED' if not self.get_parameter('enabled').value else 'IDLE')

    def _publish_state(self, value):
        message = String()
        message.data = value
        self._state_pub.publish(message)

    def _configure(self):
        if self._gpio is not None:
            return
        if not bool(self.get_parameter('enabled').value):
            raise RuntimeError('蜂鸣器 GPIO 尚未启用')
        pin = int(self.get_parameter('pin').value)
        if pin < 1:
            raise RuntimeError('蜂鸣器 GPIO pin 尚未配置')
        try:
            import Jetson.GPIO as GPIO
        except ImportError as exc:
            raise RuntimeError('Orin 上未安装 Jetson.GPIO') from exc
        numbering = str(self.get_parameter('pin_numbering').value).upper()
        modes = {'BOARD': GPIO.BOARD, 'BCM': GPIO.BCM, 'TEGRA_SOC': GPIO.TEGRA_SOC}
        if numbering not in modes:
            raise RuntimeError('pin_numbering 必须是 BOARD、BCM 或 TEGRA_SOC')
        GPIO.setwarnings(False)
        GPIO.setmode(modes[numbering])
        inactive = GPIO.LOW if bool(self.get_parameter('active_high').value) else GPIO.HIGH
        GPIO.setup(pin, GPIO.OUT, initial=inactive)
        self._gpio = GPIO
        self._configured_pin = pin

    def _on_command(self, message):
        duration_ms = int(message.data)
        if not 100 <= duration_ms <= 2000:
            self.get_logger().warning('拒绝越界蜂鸣器时长')
            return
        if not self._lock.acquire(blocking=False):
            self.get_logger().warning('蜂鸣器正忙，忽略重叠命令')
            return
        threading.Thread(
            target=self._buzz, args=(duration_ms,), daemon=True).start()

    def _buzz(self, duration_ms):
        try:
            self._configure()
            active_high = bool(self.get_parameter('active_high').value)
            active = self._gpio.HIGH if active_high else self._gpio.LOW
            inactive = self._gpio.LOW if active_high else self._gpio.HIGH
            self._publish_state('BUZZING')
            self._gpio.output(self._configured_pin, active)
            time.sleep(duration_ms / 1000.0)
            self._gpio.output(self._configured_pin, inactive)
            self._publish_state('IDLE')
        except Exception as exc:
            self.get_logger().error(str(exc))
            self._publish_state(f'ERROR: {exc}')
        finally:
            self._lock.release()

    def destroy_node(self):
        if self._gpio is not None and self._configured_pin is not None:
            inactive = self._gpio.LOW if bool(
                self.get_parameter('active_high').value) else self._gpio.HIGH
            self._gpio.output(self._configured_pin, inactive)
            self._gpio.cleanup(self._configured_pin)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BuzzerGpioNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
