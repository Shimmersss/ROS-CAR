"""Check sync boundaries without opening an SSH connection."""
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('sync', ROOT / 'scripts/sync_to_jetson.py')
sync = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sync)


class SyncTests(unittest.TestCase):
    def invoke(self, extra):
        argv = ['sync', '--host', 'user@192.168.1.10', '--dest', '/home/user/ROSCAR'] + extra
        with patch('sys.argv', argv), patch.object(sync.subprocess, 'run') as run, \
                patch.object(sync, 'route_of', return_value=('en0', '192.168.1.1')):
            sync.main()
            return run.call_args.args[0]

    def test_default_is_preview_without_bulk_assets(self):
        command = self.invoke([])
        self.assertIn('--dry-run', command)
        self.assertNotIn('--delete', command)
        self.assertIn('ros2_ws/src', command)
        self.assertNotIn('models/weights', command)
        self.assertNotIn('data/recordings', command)
        self.assertFalse(any('JP6.2' in value for value in command))

    def test_apply_is_explicit(self):
        self.assertNotIn('--dry-run', self.invoke(['--apply']))

    def test_unsafe_destination_cannot_reach_rsync(self):
        with patch('sys.argv', ['sync', '--host', 'user@host', '--dest', '/home/user/../other']), \
                patch.object(sync.subprocess, 'run') as run:
            with self.assertRaises(SystemExit):
                sync.main()
            run.assert_not_called()

    def test_proxy_tunnel_route_stops_before_rsync(self):
        argv = ['sync', '--host', 'user@192.168.1.10', '--dest', '/home/user/ROSCAR']
        for route in (('utun6', '198.18.0.1'), ('en0', '198.18.0.1')):
            with patch('sys.argv', argv), patch.object(sync.subprocess, 'run') as run, \
                    patch.object(sync, 'route_of', return_value=route):
                with self.assertRaises(SystemExit) as stop:
                    sync.main()
                self.assertIn('TUN', str(stop.exception.code))
                run.assert_not_called()
        with patch('sys.argv', argv + ['--allow-tunnel']), patch.object(sync.subprocess, 'run') as run, \
                patch.object(sync, 'route_of', return_value=('utun6', '198.18.0.1')):
            sync.main()
            run.assert_called_once()

    def test_rsync_failure_is_a_short_message(self):
        argv = ['sync', '--host', 'user@192.168.1.10', '--dest', '/home/user/ROSCAR']
        failure = sync.subprocess.CalledProcessError(255, ['rsync'])
        with patch('sys.argv', argv), patch.object(sync.subprocess, 'run', side_effect=failure), \
                patch.object(sync, 'route_of', return_value=('en0', '192.168.1.1')):
            with self.assertRaises(SystemExit) as stop:
                sync.main()
            self.assertIn('255', str(stop.exception.code))


if __name__ == '__main__':
    unittest.main()
