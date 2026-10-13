"""Execute the B entry with isolated install markers; no ROS or hardware is started."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class BInstallTest(unittest.TestCase):
    def test_root_install_wins_over_stale_workspace_and_override_is_honored(self):
        with tempfile.TemporaryDirectory(prefix='roscar-entry-') as folder:
            root = Path(folder)
            (root/'scripts').mkdir()
            script = root/'scripts/run_b_radar_foxglove.sh'
            shutil.copyfile(ROOT/'scripts'/script.name, script)
            for path, code in (('install', 42), ('ros2_ws/install', 43), ('custom', 44)):
                (root/path).mkdir(parents=True)
                (root/path/'setup.bash').write_text(f'exit {code}\n')
            # Skip only the host's /opt bootstrap. Run the real script and its path selection.
            envfile = root/'env.bash'
            envfile.write_text('source() { if [[ "$1" != /opt/ros/humble/setup.bash ]]; then builtin source "$@"; fi; }\n')
            env = dict(os.environ, BASH_ENV=str(envfile), WITH_RADAR='false')
            env.pop('PERCEPTION_INSTALL', None)
            run = lambda: subprocess.run(['bash', str(script)], env=env, capture_output=True, text=True)
            self.assertEqual(run().returncode, 42)
            env['PERCEPTION_INSTALL'] = str(root/'custom')
            self.assertEqual(run().returncode, 44)


if __name__ == '__main__':
    unittest.main()
