import unittest
import cv2
import numpy as np
from rc.geometry import reconstruct_pair
from rc.multiview import project, camera_rays, depth_image, consistent_depth, fuse_points, depth_triangles
from rc.quality import epipolar_quality


class MultiviewTests(unittest.TestCase):
    def setUp(self):
        self.K = np.array([[240., 0, 160], [0, 240., 120], [0, 0, 1]])
        self.D = np.zeros(5)

    def test_vertical_baselines_recover_known_plane(self):
        rng = np.random.default_rng(19)
        gray = cv2.resize(rng.integers(0, 256, (120, 160), np.uint8), (320, 240), interpolation=cv2.INTER_NEAREST)
        first = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        for sign in (-1, 1):
            second = cv2.warpAffine(first, np.float32([[1, 0, 0], [0, 1, -16*sign]]), (320, 240))
            a, b = np.eye(4), np.eye(4)
            b[1, 3] = sign*.2
            points, _, stats = reconstruct_pair(first, second, self.K, self.D, a, b, disparities=64)
            self.assertTrue(stats["vertical_rectification"])
            self.assertGreater(len(points), 3000)
            self.assertLess(abs(np.median(points[:, 2])-3), .025)

    def test_fast_projection_matches_opencv_with_distortion(self):
        rng = np.random.default_rng(7)
        camera = rng.uniform(-1, 1, (200, 3))
        camera[:, 2] += 4
        T = np.eye(4)
        T[:3, :3], _ = cv2.Rodrigues(np.array([.2, -.1, .3]))
        T[:3, 3] = [.4, -.2, 1.]
        points = camera @ T[:3, :3].T+T[:3, 3]
        for d in (np.array([.12, -.29, .001, .002, .15]), np.arange(8)*.001):
            expected, _ = cv2.projectPoints(camera, np.zeros(3), np.zeros(3), self.K, d)
            actual, z = project(points, T, self.K, d)
            np.testing.assert_allclose(actual, expected.reshape(-1, 2), atol=1e-8)
            np.testing.assert_allclose(z, camera[:, 2], atol=1e-8)

    def test_z_buffer_uses_nearest_visible_surface(self):
        depth = depth_image(np.array([[0, 0, 4.], [0, 0, 2.]]), np.eye(4), self.K, self.D, (240, 320))
        self.assertEqual(depth[120, 160], 2.)
        self.assertEqual(np.count_nonzero(depth), 1)

    def test_consistency_accepts_plane_and_rejects_wrong_depth(self):
        shape = (240, 320)
        rays = camera_rays(self.K, self.D, shape)
        depth = np.full(shape, 3., np.float32)
        T, other = np.eye(4), np.eye(4)
        other[0, 3] = .2
        mask, refined = consistent_depth(depth, depth, T, other, self.K, self.D, rays)
        self.assertGreater(mask.sum(), 60000)
        np.testing.assert_allclose(refined[mask], 3., atol=1e-5)
        mask, _ = consistent_depth(depth, depth*1.1, T, other, self.K, self.D, rays)
        self.assertEqual(mask.sum(), 0)
        mask, _ = consistent_depth(depth, np.zeros(shape, np.float32), T, other, self.K, self.D, rays)
        self.assertEqual(mask.sum(), 0)

    def test_consistency_rejects_round_trip_displacement(self):
        shape = (240, 320)
        rays = camera_rays(self.K, self.D, shape)
        depth = np.full(shape, 3., np.float32)
        T, other = np.eye(4), np.eye(4)
        other[0, 3] = 1.
        mask, _ = consistent_depth(depth, depth*1.009, T, other, self.K, self.D, rays,
                                   relative_tolerance=.02, pixel_tolerance=.1)
        self.assertEqual(mask.sum(), 0)

    def test_vectorized_fusion_preserves_color_and_capacity(self):
        p = np.array([[.1, .1, .1], [.2, .1, .1], [1.1, 0, 0]])
        c = np.array([[20, 40, 60], [40, 60, 80], [255, 255, 255]])
        points, colors, dropped = fuse_points(p, c, 1., capacity=1)
        self.assertEqual(dropped, 1)
        np.testing.assert_allclose(points, [[.15, .1, .1]])
        np.testing.assert_equal(colors, [[30, 50, 70]])

    def test_mesh_does_not_bridge_holes_or_depth_edges(self):
        depth = np.full((3, 3), 3., np.float32)
        valid = np.ones((3, 3), bool)
        self.assertEqual(len(depth_triangles(depth, valid)), 2)
        valid[1, 1] = False
        self.assertEqual(len(depth_triangles(depth, valid)), 0)
        valid[:] = True
        depth[:, 2] = 4.
        self.assertEqual(len(depth_triangles(depth, valid)), 0)

    def test_fusion_remaps_mesh_vertices_after_downsampling(self):
        p = np.array([[.1, .1, .1], [.2, .1, .1], [1.1, 0, 0]])
        c = np.ones_like(p)
        vertices, _, _, remap = fuse_points(p, c, 1., return_index=True)
        self.assertEqual(remap[0], remap[1])
        self.assertNotEqual(remap[0], remap[2])
        np.testing.assert_allclose(vertices[remap[0]], [.15, .1, .1])

    def test_pair_check_rejects_matches_inconsistent_with_pose(self):
        rng = np.random.default_rng(83)
        pixels = rng.uniform([50, 50], [260, 190], (60, 2))
        descriptors = rng.integers(0, 256, (60, 32), np.uint8)
        first = {"pixels": pixels, "descriptors": descriptors}
        second = {"pixels": pixels-[16, 0], "descriptors": descriptors.copy()}
        T, other = np.eye(4), np.eye(4)
        other[0, 3] = .2
        self.assertEqual(epipolar_quality(first, second, self.K, T, other)["inliers"], 60)
        other[0, 3], other[1, 3] = 0, .2
        self.assertEqual(epipolar_quality(first, second, self.K, T, other)["inliers"], 0)


if __name__ == "__main__":
    unittest.main()
