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


class OverlapMaskTests(unittest.TestCase):
    def pair(self,front_depth=None):
        image=np.full((200,260,3),200,np.uint8)
        me=NS(confidence=.9,box=(20.,20.,120.,190.),keypoints_2d=[NS(x=70.,y=60.) for _ in range(17)],
              keypoint_confidences=[.9]*17)
        for i,xy in {0:(70.,35.),5:(45.,60.),6:(95.,60.),11:(50.,120.),12:(90.,120.),13:(50.,155.),14:(90.,155.),
                     15:(50.,185.),16:(90.,185.)}.items():me.keypoints_2d[i]=NS(x=xy[0],y=xy[1])
        other=copy.deepcopy(me);other.box=(95.,25.,195.,180.)
        for k in other.keypoints_2d:k.x+=75.;k.y-=5.
        image[:, :, 1]=np.tile(np.arange(260,dtype=np.uint8),(200,1))  # texture for the blur check
        return image,me,other

    def test_overlap_masked_only_when_enabled(self):
        image,me,other=self.pair()
        self.assertEqual(appearance_crop(image,me,[me,other])[1],'overlapping people')
        crop,reason=appearance_crop(image,me,[me,other],mask_overlap=True)
        self.assertEqual(reason,'pose-masked overlapping crop')
        self.assertTrue((crop[:,-10:]==(124,116,104)).all(axis=2).any())  # neighbour's arm area greyed
        self.assertFalse((crop[60:110,30:60]==(124,116,104)).all(axis=2).any())  # own torso kept

    def test_nearer_occluder_over_torso_still_rejected(self):
        image,me,other=self.pair()
        other.box=(25.,22.,125.,192.)  # standing almost exactly in front, lower box bottom = nearer
        for a,b in zip(other.keypoints_2d,me.keypoints_2d):a.x,a.y=b.x+5.,b.y+2.
        self.assertEqual(appearance_crop(image,me,[me,other],mask_overlap=True),(None,'overlapping people'))
