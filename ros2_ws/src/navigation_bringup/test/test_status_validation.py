"""Malformed navigation state must stop requests without killing the adapter."""
import json
import unittest
import rclpy
from std_msgs.msg import String
from navigation_bringup.velocity_adapter import VelocityAdapter

class StatusValidation(unittest.TestCase):
    @classmethod
    def setUpClass(cls):rclpy.init()
    @classmethod
    def tearDownClass(cls):rclpy.shutdown()
    def test_malformed_state_rejected(self):
        node=VelocityAdapter()
        try:
            good=dict(allow_motion=True,fault=False,stamp_ns=node.get_clock().now().nanoseconds)
            for bad in [None,[],{'allow_motion':True},dict(good,stamp_ns=float('inf')),
                        dict(good,stamp_ns=True),dict(good,stamp_ns='1'),dict(good,fault='false'),
                        dict(good,stamp_ns=1),dict(good,allow_motion=1)]:
                node.on_state(String(data=json.dumps(bad)))
                self.assertIsNone(node.status)
            node.on_state(String(data=json.dumps(good)))
            self.assertIsNotNone(node.status)
        finally:node.destroy_node()
