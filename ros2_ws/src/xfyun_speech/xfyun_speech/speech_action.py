"""Optional structured speech endpoint; uses the existing single audio queue."""
import queue
import threading
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from execution_mode_interfaces.action import SpeakText
from .speech_jobs import SpeechJob


class SpeechActions:
    def __init__(self, node):
        self.node = node
        self.lock = threading.Lock()
        self.reserved = 0
        self.jobs = {}
        self.reservations = {}
        self.current = None
        self.server = ActionServer(node, SpeakText, '/voice/speak', self.execute,
                                   goal_callback=self.goal, cancel_callback=self.cancel,
                                   callback_group=ReentrantCallbackGroup())

    def goal(self, request):
        with self.lock:
            if (self.node._stop.is_set() or not request.utterance_id or len(request.utterance_id) > 128
                    or request.utterance_id in self.reservations or not request.text.strip()
                    or len(request.text) > int(self.node.get_parameter('max_text_chars').value) or self.reserved >= 2):
                return GoalResponse.REJECT
            self.reservations[request.utterance_id] = threading.Event()
            self.reserved += 1
            return GoalResponse.ACCEPT

    def cancel(self, handle):
        with self.lock:
            job = self.jobs.get(bytes(handle.goal_id.uuid))
            event = self.reservations.get(handle.request.utterance_id)
            if event:
                event.set()
            if job:
                job.cancel.set()
                if self.current is job:
                    self.node._abort_audio()
        return CancelResponse.ACCEPT

    def execute(self, handle):
        key = bytes(handle.goal_id.uuid)
        with self.lock:
            job = SpeechJob(handle.request.text.strip(), handle.request.utterance_id,
                            cancel=self.reservations[handle.request.utterance_id])
            self.jobs[key] = job
        try:
            if handle.is_cancel_requested or job.cancel.is_set():
                job.cancel.set(); job.status = 'cancelled'; job.complete.set()
            else:
                try:
                    self.node._queue.put_nowait(job)
                except queue.Full:
                    job.status, job.detail = 'rejected', 'Speech queue is full'
                    job.complete.set()
            handle.publish_feedback(SpeakText.Feedback(phase=job.status))
            while rclpy.ok() and not job.complete.wait(.05):
                if handle.is_cancel_requested:
                    self.cancel(handle)
            if not job.complete.is_set():
                job.cancel.set(); job.status = 'cancelled'
            if handle.is_cancel_requested:
                handle.canceled()
            elif job.status == 'completed':
                handle.succeed()
            else:
                handle.abort()
            return SpeakText.Result(status=job.status, detail=job.detail)
        finally:
            with self.lock:
                self.jobs.pop(key, None)
                self.reservations.pop(handle.request.utterance_id, None)
                self.reserved -= 1

    def stop(self):
        with self.lock:
            for cancel in self.reservations.values():
                cancel.set()
            for job in self.jobs.values():
                job.cancel.set()
                job.status = 'cancelled'
                job.complete.set()
            self.node._abort_audio()
