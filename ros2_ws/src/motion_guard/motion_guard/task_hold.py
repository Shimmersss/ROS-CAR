"""Stationary lease shared by execution modes. Expiry never resumes motion."""
import math
import secrets


class StationaryLease:
    def __init__(self):
        self.owner = self.session_id = self.token = ''
        self.until = 0.

    def active(self, now):
        return bool(self.token) and now < self.until

    def expire(self, now):
        if self.token and now >= self.until:
            self.owner = self.session_id = self.token = ''
            self.until = 0.
            return True
        return False

    def acquire(self, owner, session, ttl, now):
        self.expire(now)
        if not owner or len(owner) > 80 or not session or len(session) > 80:
            raise ValueError('Invalid hold owner/session')
        self._ttl(ttl)
        if self.active(now):
            raise ValueError('Another stationary lease is already active')
        self.owner, self.session_id = owner, session
        self.token = secrets.token_hex(16)
        self.until = now+ttl
        return self.token

    @staticmethod
    def _ttl(ttl):
        if not math.isfinite(ttl) or not .5 <= ttl <= 10.:
            raise ValueError('Lease must be between 0.5 and 10 seconds')

    def renew(self, token, ttl, now):
        self._ttl(ttl)
        if not self.active(now) or not secrets.compare_digest(token, self.token):
            raise ValueError('Stationary lease expired or token invalid')
        self.until = now+ttl

    def release(self, token):
        if not self.token or not secrets.compare_digest(token, self.token):
            raise ValueError('Stationary lease token invalid')
        self.owner = self.session_id = self.token = ''
        self.until = 0.
