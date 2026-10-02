"""No ROS/hardware needed: public C entrypoint defaults and explicit rejection."""
import json
from pathlib import Path
import os
import subprocess
import unittest

ROOT=Path(__file__).resolve().parents[1]


class Entrypoints(unittest.TestCase):
    def run_script(self,name,*args,**extra):
        env=dict(os.environ)
        for key in ('PERCEPTION_ROUTE','GEMINI_SERIAL','GEMINI_CALIBRATION','MODEL_PATH','YOLO_PYTHON'):
            env.pop(key,None)
        env.update(extra)
        return subprocess.run(['bash',str(ROOT/'scripts'/name),*args],env=env,text=True,capture_output=True,timeout=5)

    def test_help_and_serial_required(self):
        r=self.run_script('start_c.sh','--help');self.assertEqual(r.returncode,0);self.assertIn('Gemini/Pose',r.stdout)
        r=self.run_script('start_c.sh');self.assertNotEqual(r.returncode,0);self.assertIn('GEMINI_SERIAL',r.stderr)

    def test_calibration_and_model_required(self):
        r=self.run_script('start_c.sh',GEMINI_SERIAL='test');self.assertNotEqual(r.returncode,0);self.assertIn('GEMINI_CALIBRATION',r.stderr)
        r=self.run_script('start_c.sh',GEMINI_SERIAL='test',GEMINI_CALIBRATION='nonexistent',MODEL_PATH='/nonexistent/pose.engine')
        self.assertNotEqual(r.returncode,0);self.assertIn('模型',r.stderr)

    def test_configuration_and_layout(self):
        layout=json.loads((ROOT/'foxglove/c-layout.json').read_text())
        self.assertEqual(layout['configById']['RawMessages!persons']['topicPath'],'/perception/person_states')
        calibration=json.loads((ROOT/'ros2_ws/src/perception_bringup/config/gemini_AY2755200PW.json').read_text())
        self.assertFalse(calibration['registration_verified'])
        self.assertEqual(calibration['depth_intrinsic']['height'],400)
        launch=(ROOT/'ros2_ws/src/perception_bringup/launch/perception.launch.py').read_text()
        self.assertIn("'yolo_pose': ('yolo_person_tracker', 'pose_tracker')",launch)
        self.assertIn("default_value='yolo'",launch)


if __name__=='__main__':unittest.main()
