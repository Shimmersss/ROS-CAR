"""Production method regressions with controlled sockets/actions; no cloud or audio device."""
import json
import threading
import unittest
from concurrent.futures import Future
from collections import OrderedDict
from types import SimpleNamespace as NS
from unittest.mock import patch

import websocket
import xfyun_speech.asr_node as asr
import tongue_mode.node as tongue
import roscar_api.announcements as announcements


class AsrFinalityTest(unittest.TestCase):
    def run_utterance(self, final=False, truncated=False, cancelled=False):
        clock = [0.]
        published, closed, sent = [], [], []

        def monotonic():
            clock[0] += .1
            return clock[0]

        class Capture:
            count = 0
            def start(self):
                pass
            def read(self, size):
                self.count += 1
                value = 4000 if truncated or self.count == 1 else 0
                return value.to_bytes(2, 'little', signed=True) * (size // 2)
            def close(self):
                closed.append('capture')

        def receive(socket, assembler, timeout):
            assembler.consume({'code': 0, 'data': {'status': 2 if final else 1,
                'result': {'sn': 0, 'ws': [{'cw': [{'w': '前进'}]}]}}})
            if cancelled:
                node._cancel.set()
            return final

        cfg = dict(endpoint='fixture', sample_rate=16000, frame_ms=40,
            energy_threshold=2800, speech_onset_ms=40, silence_ms=40,
            speech_timeout_sec=5., max_utterance_sec=.15 if truncated else 15.,
            response_timeout_sec=.2, language='zh_cn', accent='mandarin', capture_device='fixture')
        node = NS(get_parameter=lambda key: NS(value=cfg[key]), _capture=lambda *args: Capture(),
            _stop=threading.Event(), _cancel=threading.Event(), _speaking=False,
            _publish_state=lambda msg: None, _receive_one=receive,
            _text_pub=NS(publish=lambda msg: published.append(msg.data)),
            get_logger=lambda: NS(info=lambda msg: None, warning=lambda msg: None))
        socket = NS(send=lambda data: sent.append(json.loads(data)), close=lambda: closed.append('socket'))
        error = None
        with patch.dict(asr.os.environ, dict(XFYUN_APP_ID='fixture', XFYUN_API_KEY='fixture',
                XFYUN_API_SECRET='fixture')), patch.object(asr, 'build_signed_websocket_url', return_value='fixture'), \
                patch.object(websocket, 'create_connection', return_value=socket), \
                patch.object(asr.time, 'monotonic', side_effect=monotonic):
            try:
                asr.XfyunAsrNode._listen_once(node, 'manual')
            except RuntimeError as exc:
                error = str(exc)
        self.assertEqual(closed, ['capture', 'socket'])
        return published, error, sent

    def test_partial_cannot_become_motion_text_on_final_timeout(self):
        published, error, _ = self.run_utterance()
        self.assertEqual(published, [])
        self.assertIn('最终结果等待超时', error)

    def test_complete_final_is_published_once(self):
        published, error, _ = self.run_utterance(final=True)
        self.assertEqual(published, ['前进'])
        self.assertIsNone(error)

    def test_utterance_cutoff_discards_partial(self):
        published, error, sent = self.run_utterance(truncated=True)
        self.assertEqual(published, [])
        self.assertIn('时长上限', error)
        self.assertTrue(all(frame['data']['status'] != 2 for frame in sent))

    def test_cancelled_final_cannot_be_published(self):
        published, error, _ = self.run_utterance(final=True, cancelled=True)
        self.assertEqual(published, [])
        self.assertIsNone(error)


class LateSpeechTest(unittest.TestCase):
    def test_acceptance_timeout_cancels_only_its_late_goal_in_same_capture(self):
        accepted, cancelled, clock = Future(), [], [0.]
        def monotonic():
            clock[0] += .5
            return clock[0]
        ctx = NS(session='fixture', capture_generation=1, cancel=threading.Event(),
            task_cancel=threading.Event(), speech_handle=None)
        node = NS(ctx=ctx, check=lambda *args: None, cfg={'speech_timeout_sec': 30.},
            speech=NS(server_is_ready=lambda: True, send_goal_async=lambda goal: accepted))
        node.wait_future = lambda *args: tongue.TongueMode.wait_future(node, *args)
        with patch.object(tongue.time, 'monotonic', side_effect=monotonic), \
                patch.object(tongue.time, 'sleep', return_value=None):
            with self.assertRaisesRegex(tongue.AnalysisError, '等待接口响应超时'):
                tongue.TongueMode.speak(node, '正在拍摄')
        accepted.set_result(NS(accepted=True, cancel_goal_async=lambda: cancelled.append('own_goal')))
        self.assertEqual(cancelled, ['own_goal'])
        self.assertFalse(ctx.cancel.is_set())
        self.assertFalse(ctx.task_cancel.is_set())
        self.assertIsNone(ctx.speech_handle)


class PhaseDeliveryTest(unittest.TestCase):
    def fixture(self):
        requests, events = [], []
        def send(request):
            future = Future()
            requests.append((request, future))
            return future
        node = NS(pending=OrderedDict(), current={}, inflight=None, inflight_started=0., retry_at=0.,
            client=NS(service_is_ready=lambda: True, call_async=send),
            events=NS(publish=events.append), get_logger=lambda: NS(warning=lambda msg: None))
        return node, requests, events

    def test_current_stage_waits_for_tts_startup_without_duplicate_heartbeats(self):
        node, requests, events = self.fixture()
        with patch.object(announcements.time, 'monotonic', return_value=1.):
            announcements.PhaseAnnouncements.enqueue(node, 'owner', 'UNCLAIMED', '等待主人认领')
        with patch.object(announcements.time, 'monotonic', return_value=10.):
            announcements.PhaseAnnouncements.enqueue(node, 'owner', 'UNCLAIMED', None)
            announcements.PhaseAnnouncements.tick(node)
            requests[0][1].set_result(NS(status='completed'))
            announcements.PhaseAnnouncements.tick(node)
            announcements.PhaseAnnouncements.enqueue(node, 'owner', 'UNCLAIMED', None)
            announcements.PhaseAnnouncements.tick(node)
        self.assertEqual(len(requests), 1)
        self.assertEqual(len(events), 1)

    def test_busy_response_does_not_restore_obsolete_follow_start(self):
        node, requests, _ = self.fixture()
        with patch.object(announcements.time, 'monotonic', return_value=1.):
            announcements.PhaseAnnouncements.enqueue(node, 'control', 'FOLLOW:ARMED_FOLLOW', '开始跟随')
            announcements.PhaseAnnouncements.tick(node)
            announcements.PhaseAnnouncements.enqueue(node, 'control', 'FOLLOW:FAULT', None)
            requests[0][1].set_result(NS(status='busy'))
            announcements.PhaseAnnouncements.tick(node)
        self.assertFalse(node.pending)
        self.assertEqual(len(requests), 1)

    def test_lost_service_response_cannot_wedge_observer_forever(self):
        node, requests, _ = self.fixture()
        with patch.object(announcements.time, 'monotonic', return_value=1.):
            announcements.PhaseAnnouncements.enqueue(node, 'owner', 'UNCLAIMED', '等待主人认领')
            announcements.PhaseAnnouncements.tick(node)
        with patch.object(announcements.time, 'monotonic', return_value=132.):
            announcements.PhaseAnnouncements.tick(node)
        self.assertIsNone(node.inflight)
        self.assertTrue(requests[0][1].cancelled())


if __name__ == '__main__':
    unittest.main()
