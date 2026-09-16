import unittest
import cv2
import numpy as np
from red_object_tracker.vision import detect, Selection, measure


class VisionTest(unittest.TestCase):
    def image(self, hue=0, saturation=255, value=255):
        hsv = np.zeros((100, 200, 3), np.uint8)
        hsv[20:60, 20:60] = (hue, saturation, value)
        return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

    def test_both_red_hue_ranges(self):
        for hue in (0, 9, 171, 179):
            self.assertEqual(len(detect(self.image(hue))[0]), 1)

    def test_noise_saturation_and_brightness(self):
        for hue, saturation, value in ((60,255,255), (0,50,255), (0,255,30)):
            self.assertEqual(detect(self.image(hue,saturation,value))[0], [])
        image = np.zeros((100,200,3),np.uint8); image[::5,::5,2] = 255
        self.assertEqual(detect(image)[0], [])

    def test_largest_confirm_and_stable_identity(self):
        selection = Selection()
        image = self.image(); image[20:40,130:150] = (0,0,255)
        components, _ = detect(image)
        for i in range(2):
            selection.update(components,i*.1)
            self.assertFalse(selection.target_id)
        selection.update(components,.2)
        target_id = selection.target_id
        self.assertEqual(selection.selected.box, (20,20,60,60))
        image[10:90,100:190] = (0,0,255)
        selection.update(detect(image)[0],.3)
        self.assertEqual(selection.target_id,target_id)
        self.assertEqual(selection.selected.box,(20,20,60,60))

    def test_loss_reacquire_new_id_after_timeout(self):
        selection=Selection(); components,_=detect(self.image())
        for t in (0,.1,.2): selection.update(components,t)
        first=selection.target_id
        selection.update([], .3)
        self.assertIsNone(selection.selected)
        self.assertEqual(selection.target_id,first)
        selection.update(components,.4)
        self.assertEqual(selection.target_id,first)
        selection.update([],1.5)
        self.assertFalse(selection.target_id)
        for t in (1.6,1.7,1.8): selection.update(components,t)
        self.assertNotEqual(selection.target_id,first)

    def test_confirmation_requires_consecutive_matches(self):
        selection=Selection(); components,_=detect(self.image())
        for t in (0,.1): selection.update(components,t)
        selection.update([], .2)
        selection.update(components,.3)
        self.assertFalse(selection.target_id)

    def test_mask_depth_excludes_background_and_holes(self):
        components,_=detect(self.image()); mask=components[0].mask
        depth=np.full((100,200),7.,np.float32);depth[mask>0]=2.
        self.assertAlmostEqual(measure(depth,mask,(100.,100.,100.,50.))[2],2.)
        depth[25:30,25:30]=0
        self.assertAlmostEqual(measure(depth,mask,(100.,100.,100.,50.))[2],2.)
        depth[mask>0]=np.nan
        self.assertIsNone(measure(depth,mask,(100.,100.,100.,50.)))

    def test_sparse_and_mixed_depth_rejected(self):
        components,_=detect(self.image());mask=components[0].mask
        depth=np.zeros((100,200),np.float32);depth[25:28,25:28]=2
        self.assertIsNone(measure(depth,mask,(100.,100.,100.,50.)))
        depth[mask>0]=2;depth[40:60,20:60]=5
        self.assertIsNone(measure(depth,mask,(100.,100.,100.,50.)))


if __name__ == '__main__': unittest.main()
