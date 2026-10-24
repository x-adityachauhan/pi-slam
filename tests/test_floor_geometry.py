import unittest
import numpy as np
from floor2d.geometry import fit_ground_frame,ground_intersections,distance_error


class FloorGeometryTests(unittest.TestCase):
    def setUp(self):
        x,z=np.meshgrid(np.linspace(-2,2,9),np.linspace(1,5,9))
        self.points=np.column_stack((x.ravel(),np.ones(x.size),z.ravel()))
        self.frame=fit_ground_frame(np.eye(4),[0,-1,0],.2,self.points,7)

    def test_height_sets_scale_only_with_plane(self):
        self.assertAlmostEqual(self.frame.metres_per_slam_unit,.2)
        T=np.eye(4);T[2,3]=10
        pose=self.frame.project(T,7,2)
        np.testing.assert_allclose(pose,[2,0,0],atol=1e-10)
        self.assertAlmostEqual(distance_error([0,0,0],pose,2)["absolute_error_pct"],0)

    def test_turn_and_coordinate_convention(self):
        T=np.eye(4);T[:3,:3]=[[0,0,1],[0,1,0],[-1,0,0]]
        T[0,3]=1
        np.testing.assert_allclose(self.frame.project(T,7,2),[0,-.2,-np.pi/2],atol=1e-10)

    def test_reset_lost_and_height_change_rejected(self):
        for mid,state in [(8,2),(7,3)]:
            with self.assertRaises(ValueError): self.frame.project(np.eye(4),mid,state)
        T=np.eye(4);T[1,3]=1
        with self.assertRaises(ValueError): self.frame.project(T,7,2)

    def test_insufficient_or_nonfloor_landmarks_rejected(self):
        with self.assertRaises(ValueError): fit_ground_frame(np.eye(4),[0,-1,0],.2,self.points[:3],7)
        bad=self.points.copy();bad[:,1]=np.linspace(.1,3,len(bad))
        with self.assertRaises(ValueError): fit_ground_frame(np.eye(4),[0,-1,0],.2,bad,7)

    def test_horizon_and_far_rays_unknown(self):
        K=np.array([[100,0,50],[0,100,50],[0,0,1]])
        xyz,valid=ground_intersections([[50,100],[50,50],[50,40],[50,51]],K,[0,-1,0],.2,2)
        self.assertEqual(valid.tolist(),[True,False,False,False])
        np.testing.assert_allclose(xyz[0],[0,.2,.4])
        self.assertTrue(np.isnan(xyz[1:]).all())
