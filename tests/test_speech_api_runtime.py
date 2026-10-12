#!/usr/bin/env python3
"""Real ROS service/action regression with a fake speaker; use an isolated domain."""
import os
import threading
import time
import unittest

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from execution_mode_interfaces.action import SpeakText
from roscar_interfaces.srv import SayText
from roscar_api.speech import SpeechApi


class SpeechRuntime(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if os.environ.get('ROS_DOMAIN_ID') != '186' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
            raise RuntimeError('Require isolated ROS_DOMAIN_ID=186 ROS_LOCALHOST_ONLY=1')
        rclpy.init()
        cls.api = SpeechApi(parameter_overrides=[Parameter('playback_timeout_sec', value=1.)])
        cls.client_node = Node('speech_api_test_client')
        cls.speaker = Node('speech_api_fake_speaker')
        cls.client = cls.client_node.create_client(SayText, '/voice/say')
        cls.executor = MultiThreadedExecutor(num_threads=8)
        for node in (cls.api, cls.client_node, cls.speaker):
            cls.executor.add_node(node)
        cls.thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.thread.start()
        assert cls.client.wait_for_service(timeout_sec=5.)

    @classmethod
    def tearDownClass(cls):
        cls.api.stop()
        cls.executor.shutdown(timeout_sec=3.)
        cls.thread.join(timeout=3.)
        for node in (cls.api, cls.client_node, cls.speaker):
            node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        self.server = None
        self.cancelled = threading.Event()
        self.entered = threading.Event()
        self.received = []

    def tearDown(self):
        if self.server:
            self.server.destroy()

    def await_future(self, future, timeout=5.):
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        self.assertTrue(event.wait(timeout), 'ROS future timed out')
        return future.result()

    def call(self, text):
        return self.await_future(self.client.call_async(SayText.Request(text=text)))

    def fake(self, mode='completed', accept_delay=0., play_delay=0.):
        def goal(request):
            self.received.append(request.text)
            time.sleep(accept_delay)
            return GoalResponse.REJECT if mode == 'rejected' else GoalResponse.ACCEPT
        def execute(handle):
            self.entered.set()
            deadline = time.monotonic() + play_delay
            while time.monotonic() < deadline:
                if handle.is_cancel_requested:
                    self.cancelled.set()
                    handle.canceled()
                    return SpeakText.Result(status='cancelled', detail='Cancelled this goal')
                time.sleep(.01)
            if mode == 'completed':
                handle.succeed()
            else:
                handle.abort()
            return SpeakText.Result(status=mode, detail='Fake speaker result')
        self.server = ActionServer(self.speaker, SpeakText, '/voice/speak', execute,
            goal_callback=goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())
        self.assertTrue(self.api.client.wait_for_server(timeout_sec=5.))

    def test_01_unavailable(self):
        self.assertEqual(self.call('你好').status, 'unavailable')

    def test_invalid_text(self):
        for text in ('', '  \n', '字' * 301):
            result = self.call(text)
            self.assertFalse(result.success)
            self.assertEqual(result.status, 'invalid_text')

    def test_verbatim_and_unique_id(self):
        self.fake()
        first = self.call('你好，小车。前进只是要读的文字。')
        second = self.call('第二句')
        self.assertTrue(first.success)
        self.assertEqual(first.status, 'completed')
        self.assertNotEqual(first.utterance_id, second.utterance_id)
        self.assertEqual(self.received, ['你好，小车。前进只是要读的文字。', '第二句'])

    def test_rejected(self):
        self.fake('rejected')
        self.assertEqual(self.call('你好').status, 'rejected')

    def test_playback_failed(self):
        self.fake('failed')
        result = self.call('你好')
        self.assertFalse(result.success)
        self.assertEqual(result.status, 'failed')

    def test_busy(self):
        self.fake(play_delay=.4)
        first = self.client.call_async(SayText.Request(text='第一句'))
        self.assertTrue(self.entered.wait(3.))
        self.assertEqual(self.call('第二句').status, 'busy')
        self.assertTrue(self.await_future(first).success)

    def test_timeout_cancels_only_own_goal(self):
        self.fake(play_delay=3.)
        result = self.call('你好')
        self.assertEqual(result.status, 'timeout')
        self.assertFalse(result.success)
        self.assertTrue(self.cancelled.wait(2.))

    def test_late_acceptance_cancelled(self):
        self.fake(accept_delay=1.4, play_delay=3.)
        self.assertEqual(self.call('你好').status, 'timeout')
        self.assertTrue(self.cancelled.wait(3.))


if __name__ == '__main__':
    unittest.main(verbosity=2)
