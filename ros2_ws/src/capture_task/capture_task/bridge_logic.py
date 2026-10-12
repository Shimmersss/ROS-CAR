"""Bounded deduplication and robot-receipt monotonic TTL."""
from collections import OrderedDict
import time


class CommandCache:
    def __init__(self, lifetime=600., capacity=2048):
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
        while len(self.entries) > self.capacity:
            self.entries.popitem(last=False)


def remaining_ttl(command, received, default, now=None):
    now = time.monotonic() if now is None else now
    ttl = command.get('ttl_sec', 0) or default
    return max(0., ttl - (now-received))
