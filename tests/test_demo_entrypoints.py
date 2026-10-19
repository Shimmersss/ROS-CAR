"""Execute demo wrapper with a recording stub: no ROS or hardware."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

class DemoEntry(unittest.TestCase):
    def run_wrapper(self, **overrides):
        with tempfile.TemporaryDirectory() as directory:
            scripts = Path(directory)/'scripts'; scripts.mkdir()
            shutil.copy(ROOT/'scripts/run_demo.sh', scripts)
            (scripts/'run_robot_onboard.sh').write_text(
                'env | sort\n')
            env = {k:v for k,v in os.environ.items() if not k.startswith(('FOLLOW_', 'MOTION_', 'WITH_'))}
            env.update(overrides)
            output = subprocess.check_output(['bash', str(scripts/'run_demo.sh')], env=env, text=True)
            return dict(line.split('=', 1) for line in output.splitlines() if '=' in line)

    def test_closed_default(self):
        env = self.run_wrapper()
        self.assertEqual(env['MOTION_ENABLED'], 'false')
        self.assertEqual(env['WITH_DEMO_MOTION'], 'true')
        self.assertEqual(env['REID_REACQUIRE_ENABLED'], 'false')
        self.assertEqual(env['FOLLOW_AUTO_ARM'], 'false')
        self.assertEqual(env['FOLLOW_TARGET_AUTO_RECOVER'], 'false')
        self.assertEqual(env['FOLLOW_PAUSE_ON_FALL'], 'true')
        self.assertEqual(env['FOLLOW_TARGET_GRACE_S'], '0.6')
        self.assertTrue(env['POSE_CONFIG'].endswith('pose_fixed_27cm_demo.yaml'))
        self.assertEqual(env['FOLLOW_TARGET_DISTANCE_M'], '1.3')
        self.assertEqual(env['FOLLOW_MAX_LINEAR_MPS'], '0.35')

    def test_explicit_motion_does_not_auto_arm(self):
        env = self.run_wrapper(MOTION_ENABLED='true', FOLLOW_AUTO_ARM='true', FOLLOW_TARGET_AUTO_RECOVER='true', REID_REACQUIRE_ENABLED='true')
        self.assertEqual(env['MOTION_ENABLED'], 'true')
        self.assertEqual(env['REID_REACQUIRE_ENABLED'], 'false')
        self.assertEqual(env['FOLLOW_AUTO_ARM'], 'false')
        self.assertEqual(env['FOLLOW_TARGET_AUTO_RECOVER'], 'true')

    def test_onboard_preserves_reacquire_override(self):
        prefix = (ROOT/'scripts/run_robot_onboard.sh').read_text().split('for device in ')[0]
        for value in ('false', 'true'):
            env = dict(os.environ, GEMINI_SERIAL='test', MODEL_PATH='/test',
                       YOLO_PYTHON='/test', POSE_CONFIG='/test', REID_REACQUIRE_ENABLED=value)
            with tempfile.NamedTemporaryFile(mode='w', suffix='.sh') as script:
                script.write(prefix+'\nprintf "%s" "$REID_REACQUIRE_ENABLED"'); script.flush()
                result = subprocess.check_output(['bash', script.name], env=env, text=True)
            self.assertEqual(result, value)

if __name__ == '__main__': unittest.main()
