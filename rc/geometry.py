"""Calibrated multi-view geometry; all lengths share the SLAM map's scale."""
import cv2
import numpy as np


def pose_matrix(pose):
    pose = np.asarray(pose, dtype=float)
    if pose.shape != (7,) or not np.isfinite(pose).all():
        raise ValueError("Expected finite x y z qx qy qz qw")
    x, y, z, w = pose[3:]
    norm = np.linalg.norm(pose[3:])
    if abs(norm-1) > .01:
        raise ValueError("Quaternion must be normalized")
    x, y, z, w = pose[3:]/norm
    R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                  [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                  [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, pose[:3]
    return T


def rectify_pair(left, right, K, distortion, T_w_c1, T_w_c2):
    if left.shape != right.shape or left.ndim != 3:
        raise ValueError("Camera images must have matching color dimensions")
    height, width = left.shape[:2]
    relative = np.linalg.inv(T_w_c2) @ T_w_c1
    if np.linalg.norm(relative[:3, 3]) < 1e-8:
        raise ValueError("No translation baseline; depth is unobservable")
    R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
        K, distortion, K, distortion, (width, height),
        relative[:3, :3], relative[:3, 3], flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
    if abs(P2[1, 3]) > abs(P2[0, 3]):
        raise ValueError("Vertical epipolar pair is not supported by horizontal SGBM")
    if not np.isfinite(Q).all():
        raise ValueError("Degenerate rectification")
    maps1 = cv2.initUndistortRectifyMap(K, distortion, R1, P1, (width, height), cv2.CV_32FC1)
    maps2 = cv2.initUndistortRectifyMap(K, distortion, R2, P2, (width, height), cv2.CV_32FC1)
    first = cv2.remap(left, *maps1, cv2.INTER_LINEAR)
    second = cv2.remap(right, *maps2, cv2.INTER_LINEAR)
    mask = np.full((height, width), 255, np.uint8)
    valid1 = cv2.remap(mask, *maps1, cv2.INTER_NEAREST) == 255
    valid2 = cv2.remap(mask, *maps2, cv2.INTER_NEAREST) == 255
    if min(valid1.mean(), valid2.mean()) < .2:
        raise ValueError("Insufficient rectified image coverage")
    return first, second, valid1, valid2, Q, R1, P2


def reconstruct_pair(left, right, K, distortion, T_w_c1, T_w_c2,
                     disparities=128, sample_step=2, depth_limit=None):
    if disparities <= 0 or disparities % 16 or sample_step < 1:
        raise ValueError("Disparities must be a positive multiple of 16; step >= 1")
    first, second, valid1, valid2, Q, R1, P2 = rectify_pair(
        left, right, K, distortion, T_w_c1, T_w_c2)
    gray1 = cv2.cvtColor(first, cv2.COLOR_BGR2GRAY)
    gray2 = cv2.cvtColor(second, cv2.COLOR_BGR2GRAY)
    minimum = 0 if P2[0, 3] < 0 else -disparities
    def matcher(lower):
        return cv2.StereoSGBM_create(minDisparity=lower, numDisparities=disparities,
            blockSize=5, P1=8*25, P2=32*25, disp12MaxDiff=1, uniquenessRatio=12,
            speckleWindowSize=80, speckleRange=2, preFilterCap=31,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
    disparity = matcher(minimum).compute(gray1, gray2).astype(np.float32)/16
    min_right = -minimum-disparities+1
    disparity_right = matcher(min_right).compute(gray2, gray1).astype(np.float32)/16
    yy, xx = np.indices(disparity.shape)
    xr = np.rint(xx-disparity).astype(np.int32)
    in_bounds = (xr >= 0) & (xr < disparity.shape[1])
    xr_safe = np.clip(xr, 0, disparity.shape[1]-1)
    reverse = disparity_right[yy, xr_safe]
    image_float = gray1.astype(np.float32)
    mean = cv2.boxFilter(image_float, -1, (7, 7))
    variance = cv2.boxFilter(image_float*image_float, -1, (7, 7))-mean*mean
    valid = (valid1 & in_bounds & valid2[yy, xr_safe] &
             (disparity > minimum-.5) & (reverse > min_right-.5) &
             (np.abs(disparity) >= 1) & (np.abs(disparity+reverse) <= 1.25) &
             (variance > 25))
    xyz_rectified = cv2.reprojectImageTo3D(disparity, Q)
    # Zero disparity produces infinity by definition; reject it below.
    with np.errstate(invalid="ignore"):
        xyz_camera = xyz_rectified @ R1
    valid &= np.isfinite(xyz_camera).all(axis=2) & (xyz_camera[:, :, 2] > 0)
    if depth_limit is not None:
        valid &= xyz_camera[:, :, 2] < depth_limit
    sampling = np.zeros(valid.shape, bool)
    sampling[::sample_step, ::sample_step] = True
    valid &= sampling
    camera_points = xyz_camera[valid]
    points = camera_points @ T_w_c1[:3, :3].T + T_w_c1[:3, 3]
    colors = first[valid][:, ::-1].copy()
    return points, colors, {"valid_points": len(points),
        "valid_pixel_fraction": float(valid.mean()), "baseline_map_units":
        float(np.linalg.norm(T_w_c1[:3, 3]-T_w_c2[:3, 3]))}
