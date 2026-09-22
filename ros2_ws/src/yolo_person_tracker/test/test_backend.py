import unittest
from unittest.mock import Mock
import tempfile
from yolo_person_tracker.backend import YoloBackend


class BackendTest(unittest.TestCase):
    def test_missing_weight_never_downloads(self):
        with self.assertRaises(ValueError):
            YoloBackend('/nonexistent/yolo26s.pt', 'cpu', 320)

    def backend(self):
        backend = YoloBackend.__new__(YoloBackend)
        backend.model = Mock(); backend.tracker = Mock()
        backend.device = 'cpu'; backend.image_size = 640; backend.nms_free = True
        return backend

    def test_all_raw_detections_and_original_indices(self):
        backend = self.backend()
        boxes = Mock()
        boxes.xyxy.cpu.return_value.tolist.return_value = [[1,2,30,90],[50,4,80,100]]
        boxes.conf.cpu.return_value.tolist.return_value = [.12,.8]
        backend.model.predict.return_value = [Mock(boxes=boxes)]
        backend.tracker.update.return_value = [[51,5,81,101,4,.8,0,1]]
        result = backend.infer('image')
        self.assertIsNone(result[0].track_id)
        self.assertEqual(result[1].track_id, 4)
        self.assertEqual(result[1].box, (50.,4.,80.,100.))
        backend.model.predict.assert_called_once_with('image', classes=[0],conf=.1,iou=.7,
            imgsz=640,device='cpu',nms=False,rect=False,verbose=False)
        backend.tracker.update.assert_called_once()
        backend.reset(); backend.tracker.reset.assert_called_once()

    def test_empty_detections_update_tracker(self):
        backend = self.backend(); boxes=Mock()
        boxes.xyxy.cpu.return_value.tolist.return_value=[]
        boxes.conf.cpu.return_value.tolist.return_value=[]
        backend.model.predict.return_value=[Mock(boxes=boxes)]
        backend.tracker.update.return_value=[]
        self.assertEqual(backend.infer('image'), [])
        backend.tracker.update.assert_called_once()

    def test_engine_requires_cuda_device(self):
        with tempfile.NamedTemporaryFile(suffix='.engine') as file:
            with self.assertRaisesRegex(ValueError, 'requires device=0'):
                YoloBackend(file.name, 'cpu', 640)

    def test_nms_free_cannot_silently_use_legacy_head(self):
        backend=self.backend(); backend.model.predictor.model.end2end=False
        backend.model.predict.return_value=[Mock(boxes=None)]
        with self.assertRaisesRegex(RuntimeError, 'no end-to-end output'):
            backend.infer('image')
        backend.nms_free=False
        backend.model.predict.return_value=[Mock(boxes=None)]
        with self.assertRaisesRegex(RuntimeError, 'no boxes container'):
            backend.infer('image')
        self.assertIsNone(backend.model.predict.call_args.kwargs['nms'])
