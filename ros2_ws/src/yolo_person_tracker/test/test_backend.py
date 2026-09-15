import unittest
from unittest.mock import Mock, patch
import sys
from yolo_person_tracker.backend import YoloBackend


class BackendTest(unittest.TestCase):
    def test_missing_weight_never_downloads(self):
        with self.assertRaises(ValueError):
            YoloBackend('/nonexistent/yolo11n.pt', 'cpu', 320)

    def test_tracking_options_and_outputs(self):
        model = Mock()
        boxes = Mock()
        boxes.id.cpu.return_value.tolist.return_value = [4]
        boxes.xyxy.cpu.return_value.tolist.return_value = [[1,2,30,90]]
        boxes.conf.cpu.return_value.tolist.return_value = [.8]
        model.track.return_value = [Mock(boxes=boxes)]
        with patch.dict(sys.modules, {'ultralytics': Mock(YOLO=Mock(return_value=model))}):
            backend = YoloBackend(__file__, 'cpu', 640)
        result = backend.infer('image')
        self.assertEqual(result[0].track_id, 4)
        self.assertEqual(result[0].box, (1.,2.,30.,90.))
        model.track.assert_called_once_with('image',persist=True,tracker='bytetrack.yaml',
                                           classes=[0],conf=.1,iou=.7,imgsz=640,
                                           device='cpu',verbose=False)
