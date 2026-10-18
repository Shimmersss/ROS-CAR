#!/usr/bin/env python3
"""ROS switch tests with actual subprocess memory and simulated speech, domain 194."""
import os
from pathlib import Path
import signal
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from execution_mode_interfaces.action import SpeakText
from offline_voice.asr_manager import AsrManager
from voice_command_router.router_node import VoiceCommandRouter
from xfyun_speech.tts_node import XfyunTtsNode


class SimulatedSpeech(XfyunTtsNode):
    def __init__(self):
        self.entered = threading.Event()
        self.release = threading.Event()
        super().__init__()

    def _speak(self, text):
        self.entered.set()
        self.release.wait(5.)  # No ALSA or model; exercise the production queue/action lifecycle.


class AsrSwitchRuntime(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if os.environ.get('ROS_DOMAIN_ID') != '194' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
            raise RuntimeError('Require isolated domain 194 and localhost')
        rclpy.init()
        cls.temp = tempfile.TemporaryDirectory()
        cls.script = Path(cls.temp.name)/'worker.py'
        cls.script.write_text("""import os,time
from offline_voice.asr_worker import protect_parent_lifetime
protect_parent_lifetime()
memory=bytearray(48*1024*1024)
os.write(int(os.environ['ROSCAR_ASR_READY_FD']),b'1')
time.sleep(60)
""")
        cls.manager = AsrManager(parameter_overrides=[Parameter('asr_enabled', value=False),
            Parameter('load_timeout_sec', value=1.)])
        original = cls.manager.command
        def fake_command():
            original()  # Real per-generation relays and backend/config arguments.
            def accept(request,response):
                response.success=True
                return response
            for client in cls.manager.worker_clients.values():
                cls.peer.create_service(Trigger,client.srv_name,accept)
            return [sys.executable, str(cls.script)]
        cls.command_patch = patch.object(cls.manager, 'command', side_effect=fake_command)
        cls.command_patch.start()
        cls.router = VoiceCommandRouter()
        cls.speaker = SimulatedSpeech()
        # Enable the normal typed speech server without loading any native TTS model.
        from xfyun_speech.speech_action import SpeechActions
        cls.speaker._action_speech = SpeechActions(cls.speaker)
        cls.peer = Node('asr_switch_test_client')
        cls.client = cls.peer.create_client(SetBool, '/voice/set_asr_enabled')
        cls.speech = ActionClient(cls.peer, SpeakText, '/voice/speak')
        cls.texts = []
        cls.peer.create_subscription(String, '/voice/asr_text', lambda m: cls.texts.append(m.data), 10)
        cls.executor = MultiThreadedExecutor(num_threads=8)
        for node in [cls.manager, cls.router, cls.speaker, cls.peer]:
            cls.executor.add_node(node)
        cls.thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.thread.start()
        assert cls.client.wait_for_service(timeout_sec=5.)

    @classmethod
    def tearDownClass(cls):
        cls.speaker.release.set()
        cls.manager.stop()
        cls.executor.shutdown(timeout_sec=5.)
        cls.thread.join(timeout=5.)
        for node in [cls.manager, cls.router, cls.speaker, cls.peer]:
            node.destroy_node()
        cls.command_patch.stop()
        cls.temp.cleanup()
        rclpy.shutdown()

    def wait(self, check, timeout=5.):
        end = time.monotonic()+timeout
        while time.monotonic()<end:
            if check():return
            time.sleep(.02)
        self.assertTrue(check(), 'Timed out waiting for condition')

    def future(self, future):
        self.wait(future.done, 8.)
        return future.result()

    def toggle(self, enabled):
        response = self.future(self.client.call_async(SetBool.Request(data=enabled)))
        if not response.success:
            print(f'Switch {enabled} rejected: {response.message}', flush=True)
        return response

    def tearDown(self):
        self.speaker.release.set()
        self.assertTrue(self.toggle(False).success)

    def test_01_start_disabled_allocates_no_worker(self):
        self.assertFalse(self.manager.enabled)
        self.assertEqual(self.manager.worker.pid, 0)
        self.wait(lambda:not self.router._asr_enabled)
        for service, expected in (('/voice/start_listening',False),('/voice/cancel_listening',True)):
            client=self.peer.create_client(Trigger,service)
            self.assertTrue(client.wait_for_service(timeout_sec=3.))
            self.assertEqual(self.future(client.call_async(Trigger.Request())).success,expected)

    def test_enable_off_reload_and_router_gate(self):
        self.assertTrue(self.toggle(True).success)
        self.wait(lambda:self.router._asr_enabled)
        pid, generation = self.manager.worker.pid, self.manager.generation
        self.assertGreater(self.manager.worker.rss_kib(), 48*1024)
        self.assertTrue(self.toggle(True).success)
        self.assertEqual(self.manager.worker.pid, pid)
        self.router._on_wake(String(data='小车唤醒'))
        self.assertTrue(self.router._session_active())
        self.assertTrue(self.toggle(False).success)
        self.wait(lambda:not self.router._asr_enabled)
        self.assertFalse(Path(f'/proc/{pid}').exists())
        self.assertFalse(self.router._session_active())
        self.router._on_wake(String(data='小车唤醒'))
        self.assertFalse(self.router._session_active())
        count=len(self.texts)
        self.manager.relay(String(data='前进'), generation, self.manager.text_pub)
        self.assertTrue(self.toggle(True).success)
        self.wait(lambda:self.router._asr_enabled)
        self.assertNotEqual(self.manager.worker.pid,pid)
        self.assertFalse(self.router._session_active())
        self.manager.relay(String(data='前进'),generation,self.manager.text_pub)
        time.sleep(.1)
        self.assertEqual(len(self.texts),count)

    def test_disable_keeps_inflight_speech_alive(self):
        self.assertTrue(self.toggle(True).success)
        self.speaker.entered.clear();self.speaker.release.clear()
        self.assertTrue(self.speech.wait_for_server(timeout_sec=3.))
        handle=self.future(self.speech.send_goal_async(SpeakText.Goal(utterance_id='switch-speech',text='测试播报')))
        self.assertTrue(handle.accepted)
        self.assertTrue(self.speaker.entered.wait(3.))
        result=handle.get_result_async()
        self.assertTrue(self.toggle(False).success)
        self.assertFalse(result.done())
        self.speaker.release.set()
        self.assertEqual(self.future(result).result.status,'completed')

    def test_failed_load_and_timeout_leave_no_model_process(self):
        for command in ([sys.executable,'-c','raise SystemExit(7)'],[sys.executable,'-c','import time;time.sleep(60)']):
            with patch.object(self.manager,'command',return_value=command):
                response=self.toggle(True)
                self.assertFalse(response.success)
                self.assertFalse(self.manager.enabled)
                self.assertEqual(self.manager.worker.pid,0)
                self.assertEqual(self.manager.phase,'ERROR')

    def test_worker_crash_does_not_auto_reload(self):
        self.assertTrue(self.toggle(True).success)
        pid=self.manager.worker.pid
        self.assertGreater(pid,1)
        os.kill(pid,signal.SIGKILL)
        self.wait(lambda:self.manager.phase=='ERROR')
        self.wait(lambda:not self.router._asr_enabled)
        self.assertFalse(self.manager.enabled)
        time.sleep(.3)
        self.assertEqual(self.manager.worker.pid,0)
        self.assertTrue(self.toggle(True).success)

    def test_off_interrupts_model_loading(self):
        with patch.object(self.manager,'command',return_value=[sys.executable,'-c','import time;time.sleep(60)']):
            loading=self.client.call_async(SetBool.Request(data=True))
            self.wait(lambda:self.manager.phase=='LOADING')
            self.assertTrue(self.toggle(False).success)
            self.assertFalse(self.future(loading).success)
            self.assertEqual(self.manager.phase,'DISABLED')
            self.assertEqual(self.manager.worker.pid,0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
