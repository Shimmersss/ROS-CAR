"""Check delayed stop acknowledgements against wake/cancel/state races."""
import json
from pathlib import Path
import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.parameter import Parameter
from rclpy.task import Future
from std_msgs.msg import String
from std_srvs.srv import Trigger
from voice_command_router.router_node import VoiceCommandRouter


class DeferredStop:
    def __init__(self):
        self.future = Future()

    def service_is_ready(self):
        return True

    def call_async(self, request):
        return self.future


class ModePublisher:
    def __init__(self):
        self.messages = []

    def get_subscription_count(self):
        return 1

    def publish(self, message):
        self.messages.append(message)


@pytest.fixture
def bridge():
    rclpy.init()
    catalog = Path(__file__).resolve().parents[2]/'execution_mode_manager/config/modes.yaml'
    node = VoiceCommandRouter(parameter_overrides=[Parameter('execution_modes_config', value=str(catalog))])
    stop, publisher = DeferredStop(), ModePublisher()
    node._control_services['STOP'], node._mode_pub = stop, publisher
    try:
        yield node, stop, publisher
    finally:
        node.destroy_node()
        rclpy.shutdown()


def state(mode, phase='READY'):
    return String(data=json.dumps(dict(active=True, mode_id=mode, phase=phase)))


def test_rewake_and_old_mode_feedback_do_not_lose_pending_entry(bridge):
    node, stop, publisher = bridge
    node._send_mode('tongue', 'ENTER')
    node._on_wake(String(data='小车唤醒'))
    node._on_execution_state(state('other'))
    assert node._execution_pending and not publisher.messages
    stop.future.set_result(Trigger.Response(success=True))
    assert [(m.mode_id, m.operation) for m in publisher.messages] == [('tongue', 'ENTER')]
    node._on_execution_state(state('tongue', 'STARTING'))
    assert not node._execution_pending and node._execution_active


def test_exit_before_stop_ack_never_dispatches_obsolete_enter(bridge):
    node, stop, publisher = bridge
    node._send_mode('tongue', 'ENTER')
    node._request_mode_exit()
    stop.future.set_result(Trigger.Response(success=True))
    assert all(m.operation != 'ENTER' for m in publisher.messages)


def test_ready_heartbeat_does_not_keep_listening_forever(bridge):
    node, _, _ = bridge
    node._mode_voice_session = True
    node._on_execution_state(state('tongue'))
    assert node._session_active()
    node.session.deadline = -float('inf')
    node._on_execution_state(state('tongue'))
    assert not node._session_active()
