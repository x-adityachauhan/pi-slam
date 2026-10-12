"""Image and calibrated epipolar checks for recorded camera pairs."""
from collections import OrderedDict
import cv2
import numpy as np


class ImageCache:
    def __init__(self, K, distortion, capacity=80):
        self.K, self.distortion, self.capacity = K, distortion, capacity
        self.items = OrderedDict()
        self.detector = cv2.ORB_create(nfeatures=1000, fastThreshold=10)
        self.reads = 0

    def get(self, path):
        key = str(path)
        if key not in self.items:
            image = cv2.imread(key)
            if image is None:
                raise ValueError("Recorded image is missing")
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            keypoints, descriptors = self.detector.detectAndCompute(gray, None)
            pixels = np.array([p.pt for p in keypoints], np.float32).reshape(-1, 1, 2)
            pixels = (cv2.undistortPoints(pixels, self.K, self.distortion, P=self.K)
                      .reshape(-1, 2) if len(pixels) else np.empty((0, 2)))
            self.items[key] = {"image": image, "pixels": pixels, "descriptors": descriptors,
                "sharpness": float(cv2.Laplacian(gray, cv2.CV_32F).var()),
                "brightness": float(gray.mean()), "features": len(keypoints)}
            self.reads += 1
            if len(self.items) > self.capacity:
                self.items.popitem(last=False)
        self.items.move_to_end(key)
        return self.items[key]


def epipolar_quality(first, second, K, T1, T2, threshold=2.):
    """Check descriptor matches against the supplied poses, without refitting them."""
    a, b = first["descriptors"], second["descriptors"]
    if a is None or b is None or min(len(a), len(b)) < 2:
        return {"matches": 0, "inliers": 0, "median_error_px": None, "inlier_fraction": 0.}
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
    forward = matcher.knnMatch(a, b, k=2)
    backward = matcher.knnMatch(b, a, k=2)
    reverse = {pair[0].queryIdx: pair[0].trainIdx for pair in backward
               if len(pair) == 2 and pair[0].distance < .8*pair[1].distance}
    matches = [pair[0] for pair in forward if len(pair) == 2
               and pair[0].distance < .8*pair[1].distance
               and reverse.get(pair[0].trainIdx) == pair[0].queryIdx]
    if not matches:
        return {"matches": 0, "inliers": 0, "median_error_px": None, "inlier_fraction": 0.}
    p = np.column_stack(([first["pixels"][m.queryIdx] for m in matches], np.ones(len(matches))))
    q = np.column_stack(([second["pixels"][m.trainIdx] for m in matches], np.ones(len(matches))))
    relative = np.linalg.inv(T2) @ T1
    x, y, z = relative[:3, 3]
    cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    inverse = np.linalg.inv(K)
    F = inverse.T @ cross @ relative[:3, :3] @ inverse
    lines2, lines1 = p @ F.T, q @ F
    residual = np.abs(np.sum(q*lines2, axis=1))
    distance = .5*residual*(1/np.maximum(np.linalg.norm(lines2[:, :2], axis=1), 1e-12)
                             + 1/np.maximum(np.linalg.norm(lines1[:, :2], axis=1), 1e-12))
    return {"matches": len(matches), "inliers": int(np.sum(distance <= threshold)),
            "median_error_px": float(np.median(distance)),
            "inlier_fraction": float(np.mean(distance <= threshold))}
