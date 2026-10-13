"""Monotonic freshness for shared mode and capture health snapshots."""
import time

# tongue_mode exits after 6 seconds without manager heartbeats; allow cleanup.
MODE_MAX_AGE = 6.5
# The capture executor normally publishes every 0.25 seconds.
EXECUTOR_MAX_AGE = 1.5


def fresh(received, max_age, now=None):
    now = time.monotonic() if now is None else now
    return received > 0 and 0 <= now-received <= max_age


def mode_active(state, received, now=None):
    return fresh(received, MODE_MAX_AGE, now) and state.get('active') is True
