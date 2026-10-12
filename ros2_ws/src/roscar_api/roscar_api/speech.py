"""Public text-to-speech service backed by the existing single TTS action queue."""
import math
import threading
import time
import uuid

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from execution_mode_interfaces.action import SpeakText
from roscar_interfaces.srv import SayText


class SpeechApi(Node):
    def __init__(self, **kwargs):
        super().__init__('speech_api', **kwargs)
        self.declare_parameter('playback_timeout_sec', 120.0)
        self.timeout = float(self.get_parameter('playback_timeout_sec').value)
        if not math.isfinite(self.timeout) or not 1 <= self.timeout <= 300:
            raise ValueError('playback_timeout_sec must be between 1 and 300')
        self._busy = threading.Lock()
        self._stopping = threading.Event()
        self._goal = None
        group = ReentrantCallbackGroup()
        self.client = ActionClient(self, SpeakText, '/voice/speak', callback_group=group)
        self.service = self.create_service(SayText, '/voice/say', self.say, callback_group=group)

    def wait(self, future, deadline):
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        while not done.is_set():
            remaining = deadline - time.monotonic()
            if self._stopping.is_set() or remaining <= 0:
                raise TimeoutError('Timed out; cancellation requested for this utterance only')
            done.wait(min(.05, remaining))
        return future.result()

    @staticmethod
    def cancel_when_accepted(future):
        # A server may accept after the service has already timed out.
        try:
            goal = future.result()
            if goal is not None and goal.accepted:
                goal.cancel_goal_async()
        except Exception:
            pass  # ROS shutdown can make late cancellation unavailable.

    def say(self, request, response):
        response.success = False
        if not request.text.strip() or len(request.text) > 300:
            response.status, response.message = 'invalid_text', 'Require nonempty text, at most 300 characters'
            return response
        if not self._busy.acquire(blocking=False):
            response.status, response.message = 'busy', 'Another /voice/say request is still running'
            return response
        pending, goal = None, None
        try:
            response.utterance_id = 'say-' + uuid.uuid4().hex
            if self._stopping.is_set() or not self.client.wait_for_server(timeout_sec=2.):
                response.status, response.message = 'unavailable', '/voice/speak is unavailable; enable the existing TTS action'
                return response
            deadline = time.monotonic() + self.timeout
            pending = self.client.send_goal_async(SpeakText.Goal(
                utterance_id=response.utterance_id, text=request.text))
            goal = self.wait(pending, deadline)
            if not goal.accepted:
                response.status, response.message = 'rejected', 'TTS rejected the request (busy, stopping or text limit)'
                return response
            self._goal = goal
            result = self.wait(goal.get_result_async(), deadline)
            response.success = result.status == 4 and result.result.status == 'completed'
            response.status = result.result.status if not response.success else 'completed'
            if response.status == 'completed' and not response.success:
                response.status = 'failed'
            response.message = result.result.detail or ('Playback completed' if response.success else 'Playback did not complete')
        except TimeoutError as exc:
            if pending is not None:
                pending.add_done_callback(self.cancel_when_accepted)
            response.status, response.message = 'timeout', str(exc)
        except Exception as exc:
            if pending is not None:
                pending.add_done_callback(self.cancel_when_accepted)
            response.status, response.message = 'failed', str(exc)[:200]
        finally:
            self._goal = None
            self._busy.release()
        return response

    def stop(self):
        self._stopping.set()
        if self._goal is not None and rclpy.ok(context=self.context):
            try:
                self._goal.cancel_goal_async()
            except RuntimeError:
                pass  # Context may be closing concurrently.


def main(args=None):
    rclpy.init(args=args)
    node = SpeechApi()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        executor.shutdown(timeout_sec=3.)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
