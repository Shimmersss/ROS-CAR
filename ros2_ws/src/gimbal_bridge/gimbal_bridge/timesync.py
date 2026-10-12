"""MCU microsecond clock -> host (ROS) time, NTP-style with minimum-RTT selection."""
from collections import deque
import math

import numpy as np

WRAP = 1 << 32


class Unwrapper:
    """Extend the MCU's wrapping uint32 microsecond counter (wraps every ~71.6 min)."""

    def __init__(self):
        self.last = None
        self.turns = 0

    def __call__(self, raw):
        raw = int(raw) & (WRAP-1)
        if self.last is None:
            self.last = raw
            return raw*1e-6
        if (raw-self.last) % WRAP < WRAP//2:
            if raw < self.last:
                self.turns += 1
            self.last = raw
            return (self.turns*WRAP+raw)*1e-6
        # Slightly older (out-of-order) sample; it may predate the last counted wrap.
        turns = self.turns-1 if raw > self.last else self.turns
        return (turns*WRAP+raw)*1e-6


class ClockSync:
    """host = mcu*(1+drift) + offset, fitted on the lowest-RTT exchanges of a sliding window.

    Each exchange: host send T1, MCU receive t_rx, MCU transmit t_tx, host receive T4.
    """

    def __init__(self, window_s=10., min_samples=3, max_rtt_s=.02, keep_best=8, step_s=.01):
        self.window_s, self.min_samples = window_s, min_samples
        self.max_rtt_s, self.keep_best, self.step_s = max_rtt_s, keep_best, step_s
        self.samples = deque()
        self.model = None
        self.best_rtt = math.inf
        self.steps = 0
        self.anchored = False

    def reset(self):
        self.samples.clear()
        self.model = None
        self.best_rtt = math.inf
        self.anchored = False

    def add(self, t1, t_rx, t_tx, t4):
        rtt = (t4-t1)-(t_tx-t_rx)
        if not all(map(math.isfinite, (t1, t_rx, t_tx, t4))) or rtt < 0 or t_tx < t_rx:
            return False
        mcu_mid, host_mid = .5*(t_rx+t_tx), .5*(t1+t4)
        stepped = bool(self.samples) and t4 <= self.samples[-1][0]
        if self.model is not None and rtt <= self.max_rtt_s:
            # A trustworthy exchange far from the model means the host clock was stepped
            # (NTP after boot, VM time sync) or the MCU restarted: re-anchor at once.
            stepped |= abs(host_mid-self.to_host(mcu_mid)) > self.step_s+rtt/2
        if stepped:
            self.reset()
            self.steps += 1
            if rtt <= self.max_rtt_s:
                self.samples.append((t4, mcu_mid, host_mid, rtt))
                self.model, self.best_rtt = (1., host_mid, mcu_mid), rtt
                self.anchored = True
                return True
        self.samples.append((t4, mcu_mid, host_mid, rtt))
        while self.samples and t4-self.samples[0][0] > self.window_s:
            self.samples.popleft()
        self._fit()
        return True

    def _fit(self):
        usable = [s for s in self.samples if s[3] <= self.max_rtt_s]
        if len(usable) < self.min_samples and self.anchored:
            # After a re-anchor, keep the step-corrected offset until enough samples exist.
            self.model = (1., float(np.mean([s[2]-s[1] for s in usable]))+usable[0][1], usable[0][1])
            self.best_rtt = min(s[3] for s in usable)
            return
        self.anchored = False
        if len(usable) < self.min_samples:
            self.model = None
            self.best_rtt = min((s[3] for s in self.samples), default=math.inf)
            return
        usable.sort(key=lambda s: s[3])
        best = usable[:self.keep_best]
        self.best_rtt = best[0][3]
        mcu = np.array([s[1] for s in best])
        host = np.array([s[2] for s in best])
        if np.ptp(mcu) >= 2.:
            # Fit drift only with enough time span; otherwise a pure offset is better conditioned.
            slope, intercept = np.polyfit(mcu-mcu.mean(), host, 1)
            if abs(slope-1.) > 1e-3:   # >1000 ppm: not a crystal, reject
                slope, intercept = 1., float(np.mean(host-mcu))+mcu.mean()
            self.model = (float(slope), float(intercept), float(mcu.mean()))
        else:
            self.model = (1., float(np.mean(host-mcu)+mcu.mean()), float(mcu.mean()))

    @property
    def synced(self):
        return self.model is not None

    def offset(self):
        if self.model is None:
            return math.nan
        slope, intercept, centre = self.model
        return intercept-centre

    def to_host(self, mcu_s):
        if self.model is None:
            return None
        slope, intercept, centre = self.model
        return intercept+slope*(mcu_s-centre)
