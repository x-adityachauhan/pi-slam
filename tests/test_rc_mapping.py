import json
import tempfile
import unittest
from pathlib import Path
import cv2
import numpy as np
from rc.geometry import pose_matrix, reconstruct_pair
from rc.cloud import Cloud
from rc.dense_reconstruct import load_views, candidate_pairs
from drone.health import PoseGate


class RCMappingTests(unittest.TestCase):
    def pair(self, sign=1):
        rng = np.random.default_rng(55)
        gray = cv2.resize(rng.integers(0, 256, (120, 160), dtype=np.uint8), (320, 240),
                          interpolation=cv2.INTER_NEAREST)
        first = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        second = cv2.warpAffine(first, np.float32([[1, 0, -16*sign], [0, 1, 0]]), (320, 240))
        K = np.array([[240., 0, 160], [0, 240, 120], [0, 0, 1]])
        T1 = np.eye(4)
        T1[:3, 3] = [1., .5, .2]
        T2 = T1.copy()
        T2[0, 3] += .2*sign
        return first, second, K, np.zeros(5), T1, T2

    def test_positive_baseline_recovers_known_3d_plane(self):
        points, colors, stats = reconstruct_pair(*self.pair(), disparities=64)
        self.assertGreater(len(points), 5000)
        self.assertLess(abs(np.median(points[:, 2])-3.2), .025)
        self.assertEqual(colors.dtype, np.uint8)

    def test_negative_disparity_baseline_recovers_same_plane(self):
        points, _, _ = reconstruct_pair(*self.pair(-1), disparities=64)
        self.assertGreater(len(points), 4000)
        self.assertLess(abs(np.median(points[:, 2])-3.2), .025)

    def test_camera_to_world_rotation_is_applied(self):
        pair = list(self.pair())
        pair[4][:3, :3], _ = cv2.Rodrigues(np.array([0., np.pi/2, 0.]))
        offset = np.eye(4)
        offset[0, 3] = .2
        pair[5] = pair[4] @ offset
        points, _, _ = reconstruct_pair(*pair, disparities=64)
        self.assertGreater(len(points), 4000)
        self.assertLess(abs(np.median(points[:, 0])-4.), .025)

    def test_pure_rotation_rejected(self):
        pair = list(self.pair())
        pair[-1] = pair[-2].copy()
        pair[-1][:3, :3], _ = cv2.Rodrigues(np.array([0, .1, 0]))
        with self.assertRaisesRegex(ValueError, "baseline"):
            reconstruct_pair(*pair, disparities=64)

    def test_untextured_images_produce_no_surfaces(self):
        pair = list(self.pair())
        pair[0][:] = 130
        pair[1][:] = 130
        points, _, _ = reconstruct_pair(*pair, disparities=64)
        self.assertEqual(len(points), 0)

    def test_voxel_fusion_and_capacity_are_bounded(self):
        cloud = Cloud(1., capacity=2)
        cloud.add(np.array([[.1, .2, .3], [.3, .2, .3], [1.2, 0, 0], [2.2, 0, 0]]),
                  np.array([[10, 20, 30], [30, 40, 50], [1, 2, 3], [4, 5, 6]]))
        points, colors = cloud.arrays()
        self.assertEqual(len(points), 2)
        self.assertEqual(cloud.dropped, 1)
        np.testing.assert_allclose(points[0], [.2, .2, .3])
        np.testing.assert_equal(colors[0], [20, 30, 40])

    def test_many_samples_from_one_pair_do_not_fake_multiview_support(self):
        cloud = Cloud(1.)
        cloud.add(np.array([[.1, 0, 0], [.2, 0, 0]]), np.ones((2, 3)))
        self.assertEqual(len(cloud.arrays(min_support=2)[0]), 0)
        cloud.add(np.array([[.3, 0, 0]]), np.ones((1, 3)))
        self.assertEqual(len(cloud.arrays(min_support=2)[0]), 1)

    def test_recorded_paths_cannot_escape_session(self):
        with tempfile.TemporaryDirectory() as folder:
            run = Path(folder)
            (run/"frames.csv").write_text("time_s,capture_timestamp_ns,sequence,image\n0,100,1,../secret.jpg\n")
            (run/"poses.tum").write_text("0 0 0 0 0 0 0 1\n")
            with self.assertRaisesRegex(ValueError, "escaped"):
                load_views(run, run/"poses.tum")

    def test_candidates_need_translation(self):
        views = [{"time": i*.5, "T": np.eye(4)} for i in range(4)]
        self.assertEqual(list(candidate_pairs(views, 1.)), [])
        views[1]["T"][0, 3] = .08
        self.assertTrue(list(candidate_pairs(views, 1.)))

    def event(self, stamp=1_000_000_000, result=10_000_000_000):
        return {"metric_scale": True, "tracking_state": 2, "time_sync_verified": True,
            "body_extrinsics_validated": True, "continuous_local_frame": True,
            "result_monotonic_ns": result, "receipt_to_result_ms": 30., "tracked_landmarks": 80,
            "capture_timestamp_ns": stamp, "pose_camera_world_xyzw": [0, 0, 0, 0, 0, 0, 1], "map_id": 0}

    def test_camera_only_never_passes_metric_gate(self):
        gate = PoseGate(recovery_frames=1)
        event = self.event()
        event["metric_scale"] = False
        result = gate.check(event, event["result_monotonic_ns"])
        self.assertIn("nonmetric_scale", result["reasons"])
        self.assertFalse(result["flight_output_enabled"])

    def test_gate_rejects_stale_lost_reset_and_nan_pose(self):
        for problem in ("stale", "lost", "reset", "nan"):
            gate = PoseGate(recovery_frames=1)
            first = self.event()
            self.assertTrue(gate.check(first, first["result_monotonic_ns"])["research_gate_passed"])
            second = self.event(stamp=1_050_000_000, result=10_050_000_000)
            now = second["result_monotonic_ns"]
            if problem == "stale": now += 300_000_000
            if problem == "lost": second["tracking_state"] = 3
            if problem == "reset": second["map_id"] = 1
            if problem == "nan": second["pose_camera_world_xyzw"][0] = float("nan")
            self.assertFalse(gate.check(second, now)["research_gate_passed"], problem)

    def test_gate_requires_recovery_and_rejects_position_jump(self):
        gate = PoseGate(recovery_frames=2)
        first = self.event()
        self.assertIn("recovering", gate.check(first, first["result_monotonic_ns"])["reasons"])
        second = self.event(stamp=1_050_000_000, result=10_050_000_000)
        self.assertTrue(gate.check(second, second["result_monotonic_ns"])["research_gate_passed"])
        third = self.event(stamp=1_100_000_000, result=10_100_000_000)
        third["pose_camera_world_xyzw"][0] = 3.
        self.assertIn("pose_discontinuity", gate.check(third, third["result_monotonic_ns"])["reasons"])


if __name__ == "__main__":
    unittest.main()
