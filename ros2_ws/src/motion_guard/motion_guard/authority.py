"""Single, fail-closed motion authority; no ROS dependencies."""

import json
import os
import secrets
from pathlib import Path


class AuthorityError(ValueError):
    pass


class MotionAuthority:
    STATES = {'DISARMED', 'ARMED_MANUAL', 'ARMED_FOLLOW', 'ESTOP', 'FAULT'}

    def __init__(self, latch_path=None):
        self.state = 'DISARMED'
        self.command_mode = 'IDLE'
        self.reason = 'startup_disarmed'
        self.epoch = secrets.randbits(52)  # Restart cannot reuse a prior authorization ticket.
        self.latch_path = Path(latch_path) if latch_path else None
        if self.latch_path and self.latch_path.exists():
            try:
                value = json.loads(self.latch_path.read_text())
                if not isinstance(value, dict) or set(value) != {'estop'} or value['estop'] is not False:
                    self.state, self.reason = 'ESTOP', 'persisted_estop'
            except (OSError, ValueError):
                self.state, self.reason = 'ESTOP', 'invalid_estop_record'

    @property
    def armed(self):
        return self.state in ('ARMED_MANUAL', 'ARMED_FOLLOW')

    def select_mode(self, mode):
        if mode not in ('IDLE', 'EXTERNAL', 'FOLLOW'):
            raise AuthorityError('Invalid control mode')
        if self.state in ('ESTOP', 'FAULT'):
            raise AuthorityError('Explicit reset required before selecting a mode')
        self.stop()
        self.command_mode = mode
        self.reason = 'mode_selected_disarmed'

    def arm(self, ready, enabled):
        if self.state != 'DISARMED':
            raise AuthorityError('Explicit reset/disarm required before arming')
        if enabled is not True or ready is not True or self.command_mode == 'IDLE':
            raise AuthorityError('Motion disabled or inputs not ready')
        self.state = 'ARMED_MANUAL' if self.command_mode == 'EXTERNAL' else 'ARMED_FOLLOW'
        self.reason = 'explicitly_armed'

    def stop(self):
        self.epoch += 1
        self.command_mode = 'IDLE'
        if self.state not in ('ESTOP', 'FAULT'):
            self.state = 'DISARMED'
            self.reason = 'operator_stop'

    def estop(self, reason='emergency_stop'):
        self.stop()
        self.state, self.reason = 'ESTOP', reason
        self._persist(True)

    def fault(self, reason):
        self.epoch += 1
        if self.state != 'ESTOP':
            self.state, self.reason = 'FAULT', str(reason)

    def reset_estop(self, healthy):
        if self.state != 'ESTOP' or healthy is not True:
            raise AuthorityError('ESTOP reset requires healthy emergency channel')
        self._persist(False)
        self.state, self.command_mode, self.reason = 'DISARMED', 'IDLE', 'estop_reset_disarmed'
        self.epoch += 1

    def recover_follow(self):
        """FAULT from a target loss -> ARMED_FOLLOW; the caller verified every input this tick."""
        if self.state != 'FAULT' or self.command_mode != 'FOLLOW':
            raise AuthorityError('Only a FOLLOW-mode fault can recover by itself')
        self.state, self.reason = 'ARMED_FOLLOW', 'target_recovered'
        self.epoch += 1

    def reset_fault(self, ready):
        if self.state != 'FAULT' or ready is not True:
            raise AuthorityError('FAULT reset requires recovered inputs')
        self.state, self.command_mode, self.reason = 'DISARMED', 'IDLE', 'fault_reset_disarmed'
        self.epoch += 1

    def _persist(self, latched):
        if not self.latch_path:
            return
        temporary = self.latch_path.with_suffix('.tmp')
        try:
            self.latch_path.parent.mkdir(parents=True, exist_ok=True)
            with temporary.open('w') as stream:
                json.dump({'estop': latched}, stream)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.latch_path)
            directory = os.open(self.latch_path.parent, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(directory)
            finally:
                os.close(directory)
        except OSError as exc:
            self.state, self.command_mode = 'ESTOP', 'IDLE'
            self.reason = 'estop_persistence_error'
            raise AuthorityError(self.reason) from exc
