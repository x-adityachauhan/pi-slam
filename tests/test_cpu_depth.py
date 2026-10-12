import unittest
import numpy as np
from rc.depth_worker import align_depth, Volume


class CPUDepthTests(unittest.TestCase):
    def test_inverse_depth_scale_and_shift(self):
        y,x=np.indices((120,160));pred=(x/160+y/120).astype(np.float32)
        uv=np.array([(u,v) for v in range(10,110,10) for u in range(10,150,10)])
        z=1/(pred[uv[:,1],uv[:,0]]*2+.5)
        result,error=align_depth(pred,np.column_stack((uv,z)))
        self.assertLess(error,1e-5)
        np.testing.assert_allclose(result[uv[:,1],uv[:,0]],z,rtol=1e-5)

    def test_degenerate_depth_rejected(self):
        with self.assertRaises(ValueError):
            align_depth(np.ones((120,160)),np.array([[30,40,2]]*40))

    def test_tsdf_needs_multiple_observations(self):
        volume=Volume([0,0,2],2,resolution=24)
        depth=np.full((60,80),2,np.float32);K=np.array([[60,0,40],[0,60,30],[0,0,1]])
        volume.integrate(depth,K,np.eye(4))
        self.assertEqual(volume.mesh(),([],[]))
        volume.integrate(depth,K,np.eye(4))
        points,faces=volume.mesh()
        self.assertGreater(len(faces),0)
        self.assertLess(np.max(np.abs(np.array(points)[:,2]-2)),volume.voxel)
