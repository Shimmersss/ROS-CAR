"""Bounded image-return worker, independent of command polling and speech."""
from collections import deque
import hashlib
import threading
import time
from .cloud_client import CloudClient, CloudError, Cancelled


class ImageOutbox:
    MAX_BYTES = 32 * 1024 * 1024
    MAX_ITEMS = 16

    def __init__(self, url, token, report):
        self.client = CloudClient(url, token, timeout=2.)
        self.report = report
        self.condition = threading.Condition()
        self.items = deque()
        self.bytes = 0
        self.active = 0
        self.stop_event = threading.Event()
        self.worker = threading.Thread(target=self.run, name='capture_result_images', daemon=True)
        self.worker.start()

    def put(self, task_id, capture_id, image, kind='annotated'):
        if kind not in ('annotated', 'original'):
            raise ValueError('invalid_image_kind')
        with self.condition:
            if len(self.items) + self.active >= self.MAX_ITEMS or self.bytes + len(image) > self.MAX_BYTES:
                self.report(task_id, 'image_outbox_full')
                return False
            self.items.append((task_id, capture_id, image, kind))
            self.bytes += len(image)
            self.condition.notify()
        return True

    def deliver(self, task_id, capture_id, image, kind):
        digest = hashlib.sha256(image).hexdigest()
        fields = {'Content-Type': 'image/jpeg', 'X-Capture-Id': capture_id,
                  'X-Image-Sha256': digest, 'X-Image-Kind': kind}
        for attempt in range(3):
            try:
                status, value = self.client.request('POST', '/v1/tasks/' + task_id + '/result-image',
                    image, fields, self.stop_event, time.monotonic() + 2., 2.)
                if status in (200, 409) and all(value.get(k) == v for k, v in
                    dict(status='received', task_id=task_id, capture_id=capture_id, sha256=digest).items()):
                    return
                raise CloudError(status, 'result_image_receipt')
            except CloudError as exc:
                if attempt == 2 or 400 <= exc.status < 500 and exc.status != 429:
                    raise
                if self.stop_event.wait(2 ** attempt):
                    raise Cancelled()

    def run(self):
        while not self.stop_event.is_set():
            with self.condition:
                if not self.items:
                    self.condition.wait(.1)
                    continue
                item = self.items.popleft()
                self.active = 1
                # In-flight image remains part of the byte budget.
            try:
                self.deliver(*item)
            except Cancelled:
                break
            except Exception:
                self.report(item[0], 'result_image_return_failed')
            finally:
                with self.condition:
                    self.bytes -= len(item[2])
                    self.active = 0

    def close(self):
        self.stop_event.set(); self.client.cancel()
        with self.condition:
            self.condition.notify_all()
        self.worker.join(timeout=3.)
        with self.condition:
            self.items.clear(); self.bytes = 0
