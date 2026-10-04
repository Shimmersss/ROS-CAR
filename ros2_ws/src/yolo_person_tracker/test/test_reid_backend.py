import copy
import unittest
from types import SimpleNamespace as NS
import numpy as np
from yolo_person_tracker.reid_backend import appearance_crop, OSNetBackend


def sample():
    image=np.random.default_rng(3).integers(0,256,(160,200,3),dtype=np.uint8)
    p=NS(confidence=.9,box=(20,10,100,140),keypoints_2d=[NS(x=50.,y=50.) for _ in range(17)],
         keypoint_confidences=[.9]*17)
    for i,xy in {5:(35.,35.),6:(85.,35.),11:(40.,85.),12:(80.,85.)}.items():
        p.keypoints_2d[i]=NS(x=xy[0],y=xy[1])
    return image,p


class CropTests(unittest.TestCase):
    def test_full_crop_and_rgb_normalization(self):
        image,p=sample();crop,reason=appearance_crop(image,p,[p])
        self.assertEqual(crop.shape,(130,80,3))
        tensor=OSNetBackend.tensor(np.full((20,20,3),(0,0,255),np.uint8))
        self.assertEqual(tensor.shape,(1,3,256,128))
        self.assertAlmostEqual(float(tensor[0,0,0,0]),(1-.485)/.229,places=5)

    def test_overlap_torso_missing_blur_and_truncation_reject(self):
        image,p=sample()
        other=copy.deepcopy(p)
        self.assertIsNone(appearance_crop(image,p,[p,other])[0])
        p.keypoint_confidences[5]=.1
        self.assertIsNone(appearance_crop(image,p,[p])[0])
        image,p=sample()
        self.assertIsNone(appearance_crop(np.zeros_like(image),p,[p])[0])
        p.box=(-10,0,100,140)
        self.assertIsNone(appearance_crop(image,p,[p])[0])

    def test_missing_model_errors_without_downloading(self):
        with self.assertRaises(ValueError):OSNetBackend('/nonexistent/reid.onnx')

    def test_degenerate_skeleton_and_low_detector_confidence_rejected(self):
        image,p=sample();p.keypoints_2d=[NS(x=50.,y=50.) for _ in range(17)]
        self.assertIsNone(appearance_crop(image,p,[p])[0])
        image,p=sample();p.confidence=.1
        self.assertIsNone(appearance_crop(image,p,[p])[0])
