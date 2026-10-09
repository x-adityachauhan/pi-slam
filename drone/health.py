"""Fail-closed research gate. This does NOT authorize flight or transmit data."""
import math
import time
import numpy as np


class PoseGate:
    def __init__(self, max_age=.2, max_processing_ms=100., min_landmarks=30,
                 recovery_frames=10, max_speed=15., max_angular_speed=5.):
        self.max_age, self.max_processing_ms = max_age, max_processing_ms
        self.min_landmarks, self.recovery_frames = min_landmarks, recovery_frames
        self.max_speed, self.max_angular_speed = max_speed, max_angular_speed
        self.previous = None
        self.good = 0
        self.reset_count = 0

    def check(self, event, now_ns=None):
        now_ns = time.monotonic_ns() if now_ns is None else now_ns
        reasons = []
        if event.get("metric_scale") is not True:
            reasons.append("nonmetric_scale")
        if event.get("tracking_state") != 2:
            reasons.append("tracking_unavailable")
        if event.get("time_sync_verified") is not True:
            reasons.append("camera_imu_autopilot_time_sync_unverified")
        if event.get("body_extrinsics_validated") is not True:
            reasons.append("body_extrinsics_unverified")
        if event.get("continuous_local_frame") is not True:
            reasons.append("continuous_local_frame_unverified")
        try:
            age = (now_ns-int(event["result_monotonic_ns"]))/1e9
            processing = float(event["receipt_to_result_ms"])
            if not math.isfinite(age) or age < 0 or age > self.max_age:
                reasons.append("stale_or_future_result")
            if not math.isfinite(processing) or processing < 0 or processing > self.max_processing_ms:
                reasons.append("excess_processing_age")
            if int(event["tracked_landmarks"]) < self.min_landmarks:
                reasons.append("insufficient_landmarks")
            stamp = int(event["capture_timestamp_ns"])
            pose = np.asarray(event["pose_camera_world_xyzw"], dtype=float)
            if stamp <= 0 or pose.shape != (7,) or not np.isfinite(pose).all():
                raise ValueError("invalid pose")
            qnorm = np.linalg.norm(pose[3:])
            if abs(qnorm-1) > .01:
                raise ValueError("invalid quaternion")
            map_id = int(event["map_id"])
            if self.previous is not None:
                old_stamp, old_pose, old_map = self.previous
                dt = (stamp-old_stamp)/1e9
                if map_id != old_map:
                    reasons.append("map_reset")
                    self.reset_count += 1
                elif dt <= 0:
                    reasons.append("nonmonotonic_capture")
                elif dt > self.max_age:
                    reasons.append("capture_gap")
                elif event.get("metric_scale") is True:
                    speed = np.linalg.norm(pose[:3]-old_pose[:3])/dt
                    angle = 2*np.arccos(np.clip(abs(np.dot(pose[3:], old_pose[3:])), 0, 1))
                    if speed > self.max_speed or angle/dt > self.max_angular_speed:
                        reasons.append("pose_discontinuity")
                        self.reset_count += 1
            # Rejected timestamps must not roll the baseline backwards.
            if self.previous is None or stamp > self.previous[0]:
                self.previous = stamp, pose, map_id
        except (KeyError, TypeError, ValueError, OverflowError):
            reasons.append("malformed_pose_event")
        self.good = 0 if reasons else self.good+1
        if not reasons and self.good < self.recovery_frames:
            reasons.append("recovering")
        return {"research_gate_passed": not reasons, "flight_output_enabled": False,
                "reasons": reasons, "consecutive_good_frames": self.good,
                "reset_count": self.reset_count,
                "warning": "Gate pass is not flight qualification; no vehicle output exists."}
