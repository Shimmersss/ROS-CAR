"""Explicit software-only SpeakText fake. No real TTS, audio or microphones."""
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from execution_mode_interfaces.action import SpeakText
from .catalog import speech_text


class FakeTTS(Node):
    def __init__(self, **kwargs):
        super().__init__('capture_fake_tts',**kwargs)
        self.declare_parameter('test_only',False)
        self.declare_parameter('char_seconds',.01)
        if self.get_parameter('test_only').value is not True:
            raise ValueError('fake_tts_requires_test_only_true')
        self.char_seconds=self.get_parameter('char_seconds').value
        self.received=[];self.completed=[];self.cancelled=[]
        self.server=ActionServer(self,SpeakText,'/voice/speak',self.execute,goal_callback=self.admit,
                                cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=ReentrantCallbackGroup())

    def admit(self,request):
        try:
            speech_text(request.text)
        except ValueError:
            return GoalResponse.REJECT
        if self.count_publishers('/voice/tts_playing'):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def execute(self,goal):
        self.received.append(goal.request.text)
        self.get_logger().info('TEST ONLY simulated speech: '+goal.request.text)
        end=time.monotonic()+len(goal.request.text)*self.char_seconds
        while time.monotonic()<end:
            if goal.is_cancel_requested or not rclpy.ok(context=self.context):
                self.cancelled.append(goal.request.utterance_id);goal.canceled()
                return SpeakText.Result(status='cancelled',detail='Simulated cancellation')
            time.sleep(.01)
        self.completed.append(goal.request.utterance_id);goal.succeed()
        return SpeakText.Result(status='completed',detail='Simulated; no speaker playback')


def main(args=None):
    from .runtime import run
    run(FakeTTS,args,threads=3)
