import unittest
import numpy as np
from rc.image_quality import measure


class ImageQualityTests(unittest.TestCase):
    def test_neutral_patch_and_green_cast(self):
        image=np.full((20,30,3),120,np.uint8)
        self.assertEqual(measure(image,[2,2,10,10])["neutral_channel_spread_pct"],0)
        image[:,:,1]=180
        result=measure(image,[2,2,10,10])
        self.assertAlmostEqual(result["neutral_red_to_green"],2/3)
        self.assertGreater(result["neutral_channel_spread_pct"],40)

    def test_clipping_and_invalid_roi_rejected(self):
        with self.assertRaises(ValueError):
            measure(np.full((20,30,3),255,np.uint8),[0,0,10,10])
        with self.assertRaises(ValueError):
            measure(np.full((20,30,3),100,np.uint8),[-1,0,10,10])
