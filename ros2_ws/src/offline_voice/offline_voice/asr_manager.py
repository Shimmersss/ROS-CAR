"""ROS ASR power switch; the lightweight manager never imports speech models."""
import json
import math
from pathlib import Path
import signal
import sys
import threading
import time
from types import SimpleNamespace
import uuid

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
import yaml
from ament_index_python.packages import get_package_share_directory

from .asr_process import AsrProcess


class AsrManager(Node):
    def __init__(self, closing=None, **kwargs):
        super().__init__('asr_manager', **kwargs)
        default_config = str(Path(get_package_share_directory('offline_voice'))/'config/offline_voice.yaml')
        for name, default in [('worker_config', default_config), ('use_xfyun_asr', False),
                              ('asr_enabled', True), ('load_timeout_sec', 60.)]:
            self.declare_parameter(name, default, ParameterDescriptor(read_only=True))
        self.timeout = float(self.get_parameter('load_timeout_sec').value)
        if not math.isfinite(self.timeout) or not 1 <= self.timeout <= 120:
            raise ValueError('load_timeout_sec must be 1..120')
        self.config = str(self.get_parameter('worker_config').value)
        data = yaml.safe_load(Path(self.config).read_text()) if self.config else {}
        params = dict((data or {}).get('/**', {}).get('ros__parameters', {}))
        params.update((data or {}).get('offline_asr', {}).get('ros__parameters', {}))
        self.output_topic = params.get('output_topic', '/voice/asr_text')
        self.worker = AsrProcess()
        self.closing = closing if closing is not None else threading.Event()
        self.transition = threading.Lock()
        self.loading_cancel = threading.Event()
        self.gate = threading.RLock()
        self.enabled = False
        self.phase, self.error = 'DISABLED', ''
        self.generation = ''
        self.relays = []
        self.worker_clients = {}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.enabled_pub = self.create_publisher(Bool, '/voice/asr_enabled', latched)
        self.status_pub = self.create_publisher(String, '/voice/asr_status', latched)
        self.text_pub = self.create_publisher(String, self.output_topic, 10)
        self.state_pub = self.create_publisher(String, '/voice/asr_state', 10)
        self.create_service(SetBool, '/voice/set_asr_enabled', self.set_enabled,
                            callback_group=ReentrantCallbackGroup())
        listening_group = MutuallyExclusiveCallbackGroup()  # Leave executor threads free for client replies/toggles.
        for operation in ('start', 'cancel'):
            self.create_service(Trigger, '/voice/'+('start_listening' if operation=='start' else 'cancel_listening'),
                lambda request, response, operation=operation: self.listening(operation, response),
                callback_group=listening_group)
        self.create_timer(.25, self.monitor)
        self.publish_status()
        if self.get_parameter('asr_enabled').value:
            # Default startup still waits for the same models as the old entry.
            self.change(True)

    def publish_status(self):
        if not rclpy.ok(context=self.context):
            return
        with self.gate:
            self.enabled_pub.publish(Bool(data=self.enabled))
            self.status_pub.publish(String(data=json.dumps(dict(
                enabled=self.enabled, state=self.phase, models_loaded=self.worker.ready and self.worker.alive,
                worker_pid=self.worker.pid, worker_rss_kib=self.worker.rss_kib(),
                backend='xfyun' if self.get_parameter('use_xfyun_asr').value else 'offline',
                error=self.error))))

    def relay(self, message, generation, publisher):
        with self.gate:
            if self.enabled and generation == self.generation and self.worker.alive:
                publisher.publish(message)

    def clear_relays(self):
        for subscription in self.relays:
            self.destroy_subscription(subscription)
        self.relays.clear()
        for client in self.worker_clients.values():
            self.destroy_client(client)
        self.worker_clients.clear()

    def command(self):
        self.generation = 'g' + uuid.uuid4().hex  # ROS topic tokens cannot start with a digit.
        generation = self.generation
        text = '/voice/_asr_text/' + generation
        state = '/voice/_asr_state/' + generation
        for topic, publisher in ((text, self.text_pub), (state, self.state_pub)):
            self.relays.append(self.create_subscription(String, topic,
                lambda message, pub=publisher: self.relay(message, generation, pub), 10))
        start = '/voice/_asr_start/' + generation
        cancel = '/voice/_asr_cancel/' + generation
        for operation, topic in (('start', start), ('cancel', cancel)):
            self.worker_clients[operation] = self.create_client(Trigger, topic, callback_group=ReentrantCallbackGroup())
        backend = 'xfyun' if self.get_parameter('use_xfyun_asr').value else 'offline'
        command = [sys.executable, '-m', 'offline_voice.asr_worker', '--backend', backend,
                   '--ros-args', '-r', '__node:=offline_asr']
        if self.config:
            command += ['--params-file', self.config]
        command += ['-p', 'audio_source:=topic', '-p', 'output_topic:=' + text,
                    '-r', '/voice/asr_state:=' + state,
                    '-r', '/voice/start_listening:=' + start, '-r', '/voice/cancel_listening:=' + cancel]
        if backend == 'xfyun':
            command += ['-p', 'speech_timeout_sec:=5.0', '-p', 'speech_onset_ms:=160']
        return command

    def change(self, enable):
        acquired = self.transition.acquire(timeout=.1)
        if not acquired and not enable:
            # An OFF request must be able to interrupt an expensive model reload.
            self.loading_cancel.set()
            acquired = self.transition.acquire(timeout=8.)
        if not acquired:
            return False, 'ASR is changing state; retry shortly'
        try:
            if enable and self.enabled and self.worker.alive:
                return True, 'ASR is already enabled'
            if enable and self.closing.is_set():
                return False, 'ASR manager is shutting down'
            with self.gate:
                if enable:
                    self.loading_cancel.clear()
                self.enabled = False  # Invalidate text BEFORE waiting for native decoding to stop.
                self.generation = ''
                self.phase, self.error = ('LOADING' if enable else 'DISABLING'), ''
                self.publish_status()
                self.state_pub.publish(String(data=self.phase))
            self.worker.stop()
            self.clear_relays()
            if enable:
                cancelled = SimpleNamespace(is_set=lambda: self.closing.is_set() or self.loading_cancel.is_set())
                self.worker.start(self.command(), self.timeout, cancelled)
                with self.gate:
                    self.enabled, self.phase = True, 'ENABLED'
                    self.state_pub.publish(String(data='IDLE'))
                detail = 'ASR enabled; models loaded; wake again to start recognition'
            else:
                self.phase = 'DISABLED'
                self.state_pub.publish(String(data='DISABLED'))
                detail = 'ASR disabled; worker exited and model memory released'
            self.publish_status()
            return True, detail
        except Exception as exc:
            # start() also cleans up failed or timed-out model loads.
            self.enabled, self.phase, self.error = False, 'ERROR', str(exc)[:250]
            self.clear_relays()
            self.publish_status()
            return False, self.error
        finally:
            self.transition.release()

    def set_enabled(self, request, response):
        response.success, response.message = self.change(request.data)
        return response

    def listening(self, operation, response):
        # Keep the old public services stable across worker PID/DDS changes.
        with self.gate:
            client, generation = self.worker_clients.get(operation), self.generation
            enabled = self.enabled
        if not enabled or client is None:
            response.success = operation == 'cancel'
            response.message = 'ASR is disabled or loading; no listening session'
            return response
        try:
            if not client.wait_for_service(timeout_sec=2.):
                raise RuntimeError('ASR worker service is not ready')
            future = client.call_async(Trigger.Request())
            end = time.monotonic()+3.
            while not future.done() and time.monotonic()<end and self.enabled and self.generation==generation:
                time.sleep(.01)
            if not future.done() or not self.enabled or self.generation!=generation:
                client.remove_pending_request(future)
                raise RuntimeError('ASR listening request cancelled or timed out')
            result = future.result()
            response.success, response.message = result.success, result.message
        except Exception as exc:
            response.success, response.message = False, str(exc)[:200]
        return response

    def monitor(self):
        if not self.transition.acquire(blocking=False):
            self.publish_status()
            return
        try:
            if self.enabled and not self.worker.alive:
                with self.gate:
                    self.enabled, self.phase = False, 'ERROR'
                    self.error = 'ASR worker exited; call set_asr_enabled to reload'
                    self.generation = ''
                self.state_pub.publish(String(data='ERROR: ' + self.error))
            self.publish_status()
        finally:
            self.transition.release()

    def stop(self):
        self.closing.set()
        self.loading_cancel.set()
        with self.transition:
            with self.gate:
                self.enabled, self.generation = False, ''
            self.worker.stop()
            self.clear_relays()

    def destroy_node(self):
        self.stop()
        return super().destroy_node()


def main(args=None):
    closing = threading.Event()
    previous = {sig: signal.signal(sig, lambda *_: closing.set())
                for sig in (signal.SIGINT, signal.SIGTERM)}
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = executor = thread = None
    try:
        node = AsrManager(closing=closing)
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        def spin():
            try:
                executor.spin()
            finally:
                closing.set()
        thread = threading.Thread(target=spin, daemon=True)
        thread.start()
        closing.wait()
    finally:
        if node:
            node.stop()
        if executor:
            executor.shutdown(timeout_sec=5.)
        if thread:
            thread.join(timeout=5.)
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
