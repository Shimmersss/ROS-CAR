"""Local Qwen bridge retaining the project's validated VoiceCommand route."""

import rclpy

from deepseek_ros2.chat_node import DeepSeekChatNode
from .ollama_client import OllamaClient


class OfflineChatNode(DeepSeekChatNode):
    def __init__(self):
        super().__init__()
        self.declare_parameter('local_base_url', 'http://127.0.0.1:11435')
        self.declare_parameter('local_model', 'qwen3:1.7b')
        self.declare_parameter('context_tokens', 2048)

    def _client(self):
        get = lambda name: self.get_parameter(name).value
        return OllamaClient(
            base_url=str(get('local_base_url')),
            model=str(get('local_model')),
            timeout_sec=float(get('timeout_sec')),
            max_tokens=int(get('max_tokens')),
            context_tokens=int(get('context_tokens')),
        )


def main(args=None):
    rclpy.init(args=args)
    node = OfflineChatNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
