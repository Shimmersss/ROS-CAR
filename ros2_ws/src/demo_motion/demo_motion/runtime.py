"""Demo mode policy and a systemd cgroup owner; no ROS or model imports."""
import math
from pathlib import Path
import subprocess

MODES = frozenset(('IDLE', 'FOLLOW', 'PERCEPTION', 'ROS_CONTROL', 'RELATIVE',
                   'FACE', 'FACE_UPLOAD', 'PCB', 'HELMET', 'TONGUE'))
CAPTURE_MODES = frozenset(('FACE', 'FACE_UPLOAD', 'PCB', 'HELMET', 'TONGUE'))


def validate_mode(mode, target_id=''):
    if mode not in MODES:
        raise ValueError('unknown_mode')
    if target_id and (mode != 'FOLLOW' or len(target_id) > 64 or
                      len(target_id.split(':')) != 2 or
                      not all(s.isdecimal() for s in target_id.split(':'))):
        raise ValueError('invalid_target_id')
    return mode


def manual_command(values, age, frame, max_linear=.2, max_angular=.5, radius=.4):
    """Reject unsupported axes, stale/NaN input; Ackermann yaw requires translation."""
    if (frame != 'base_link' or not math.isfinite(age) or not 0 <= age <= .2 or
            not all(math.isfinite(v) for v in values) or
            any(v != 0 for v in (values[1], values[2], values[3], values[4]))):
        return 0., 0.
    v = max(-max_linear, min(max_linear, values[0]))
    bound = min(max_angular, abs(v)/radius)
    return v, max(-bound, min(bound, values[5]))


class ModuleOwner:
    """All nested setsid children stay in one cgroup; stop waits for its removal.

    BindsTo also kills the module if the resident core exits unexpectedly.
    Script arguments are fixed mode names, never user-supplied shell text.
    """
    unit = 'roscar-demo-module.service'

    def __init__(self, root):
        self.root = Path(root).resolve()
        if not (self.root/'scripts/run_demo_module.sh').is_file():
            raise ValueError('runtime_root_missing')

    @staticmethod
    def run(args, timeout=20):
        return subprocess.run(args, check=True, capture_output=True, text=True, timeout=timeout)

    def active(self):
        result = subprocess.run(['systemctl', '--user', 'show', self.unit,
                                 '-p', 'ActiveState', '--value'], capture_output=True, text=True, timeout=3)
        return result.stdout.strip() in ('active', 'activating', 'deactivating', 'reloading')

    def stop(self):
        # systemctl stop waits for every child, including camera launch subgroups.
        result = subprocess.run(['systemctl', '--user', 'stop', self.unit],
                                capture_output=True, text=True, timeout=20)
        if result.returncode not in (0, 5):
            raise RuntimeError('module_stop_failed')
        if self.active():
            raise RuntimeError('old_module_not_reaped')

    def start(self, mode):
        validate_mode(mode)
        if self.active():
            raise RuntimeError('module_busy')
        self.run(['systemd-run', '--user', '--quiet', '--collect', '--unit='+self.unit,
                  '--property=KillMode=control-group', '--property=KillSignal=SIGINT', '--property=TimeoutStopSec=12',
                  '--property=StandardOutput=append:'+str(self.root/('artifacts/demo-runtime/module-'+mode.lower()+'.log')),
                  '--property=StandardError=inherit',
                  '--property=BindsTo=roscar-robot.service', '--property=After=roscar-robot.service',
                  '--working-directory='+str(self.root), '/bin/bash',
                  str(self.root/'scripts/run_demo_module.sh'), mode])
