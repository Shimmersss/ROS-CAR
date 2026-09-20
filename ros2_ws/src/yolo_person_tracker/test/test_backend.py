import unittest
from unittest.mock import Mock, patch
import sys
import tempfile
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
            with tempfile.NamedTemporaryFile(suffix='.pt') as file:
                backend = YoloBackend(file.name, 'cpu', 640)
        result = backend.infer('image')
        self.assertEqual(result[0].track_id, 4)
        self.assertEqual(result[0].box, (1.,2.,30.,90.))
        model.track.assert_called_once_with('image',persist=True,tracker='bytetrack.yaml',
                                           classes=[0],conf=.1,iou=.7,imgsz=640,
                                           device='cpu',nms=False,rect=False,verbose=False)

    def test_engine_requires_cuda_device(self):
        with tempfile.NamedTemporaryFile(suffix='.engine') as file:
            with self.assertRaisesRegex(ValueError, 'requires device=0'):
                YoloBackend(file.name, 'cpu', 640)

    def test_nms_free_cannot_silently_use_legacy_head(self):
        model = Mock()
        model.predictor.model.end2end = False
        model.track.return_value = [Mock(boxes=None)]
        with patch.dict(sys.modules, {'ultralytics': Mock(YOLO=Mock(return_value=model))}):
            with tempfile.NamedTemporaryFile(suffix='.pt') as file:
                backend = YoloBackend(file.name, 'cpu', 640)
                with self.assertRaisesRegex(RuntimeError, 'no end-to-end output'):
                    backend.infer('image')
                legacy = YoloBackend(file.name, 'cpu', 640, nms_free=False)
                self.assertEqual(legacy.infer('image'), [])
                self.assertIsNone(model.track.call_args.kwargs['nms'])
