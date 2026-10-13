"""Bounded deduplication and robot-receipt monotonic TTL."""
from collections import OrderedDict
import time


class CommandCache:
    def __init__(self, lifetime=600., capacity=None):
        self.entries = OrderedDict()
        self.lifetime, self.capacity = lifetime, capacity

    def get(self, key, now=None):
        now = time.monotonic() if now is None else now
        for old, (at, _) in list(self.entries.items()):
            if now-at >= self.lifetime:
                del self.entries[old]
        entry = self.entries.get(key)
        return None if entry is None else dict(entry[1])

    def put(self, key, ack, now=None):
        self.entries[key] = (time.monotonic() if now is None else now, dict(ack))
        while self.capacity is not None and len(self.entries) > self.capacity:
            self.entries.popitem(last=False)


def remaining_ttl(command, received, default, now=None):
    now = time.monotonic() if now is None else now
    ttl = command.get('ttl_sec', 0) or default
    return max(0., ttl - (now-received))


def validate_poll_wait(value):
    if type(value) is not int or not 0 <= value <= 20:
        raise ValueError('poll_wait_sec_requires_integer_0_to_20')
    return value


def poll_request(robot_id, poll_wait_sec, pending):
    wait = 0 if pending else validate_poll_wait(poll_wait_sec)
    return '/v1/robots/'+robot_id+'/commands/next?wait='+str(wait), wait+2.
