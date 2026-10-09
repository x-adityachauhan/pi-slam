#!/usr/bin/env python3
"""Collect diverse chessboard views and validate intrinsics on held-out views."""
import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np
from stream import PiStream


def settings_text(K, distortion, width, height, fps=20, features=1600):
    d = np.asarray(distortion).ravel()
    return f'''%YAML:1.0
File.version: "1.0"
Calibration.validated: 1
Camera.type: "PinHole"
Camera1.fx: {float(K[0,0]):.10f}
Camera1.fy: {float(K[1,1]):.10f}
Camera1.cx: {float(K[0,2]):.10f}
Camera1.cy: {float(K[1,2]):.10f}
Camera1.k1: {float(d[0]):.10f}
Camera1.k2: {float(d[1]):.10f}
Camera1.p1: {float(d[2]):.10f}
Camera1.p2: {float(d[3]):.10f}
Camera1.k3: {float(d[4]):.10f}
Camera.fps: {int(fps)}
Camera.RGB: 0
Camera.width: {width}
Camera.height: {height}
ORBextractor.nFeatures: {features}
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7
Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1.0
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2.0
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3.0
Viewer.ViewpointX: 0.0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -1.8
Viewer.ViewpointF: 500.0
'''


def detect(image, pattern):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCornersSB(gray, pattern, cv2.CALIB_CB_NORMALIZE_IMAGE)
    return corners.astype(np.float32) if found else None


def descriptor(corners, size):
    points = corners.reshape(-1, 2) / np.array(size)
    span = points.max(axis=0) - points.min(axis=0)
    direction = points[-1] - points[0]
    return np.array([*points.mean(axis=0), *span, np.arctan2(*direction[::-1]) / np.pi])


def solve(folder, output, pattern, square_mm, max_error):
    images, corners, descriptors = [], [], []
    size = None
    for path in sorted(folder.glob("*.jpg")):
        image = cv2.imread(str(path))
        if image is None:
            continue
        this_size = image.shape[1], image.shape[0]
        if size is not None and this_size != size:
            raise ValueError("Calibration images have different resolutions")
        size = this_size
        detected = detect(image, pattern)
        if detected is not None:
            images.append(str(path))
            corners.append(detected)
            descriptors.append(descriptor(detected, size))
    if len(corners) < 25:
        raise ValueError(f"Need at least 25 detected, varied board views; found {len(corners)}")
    desc = np.asarray(descriptors)
    occupied = {(min(2, int(x * 3)), min(2, int(y * 3))) for x, y in desc[:, :2]}
    if len(occupied) < 4:
        raise ValueError("Move the board across more of the image: need centers in at least four grid regions")
    obj = np.zeros((pattern[0] * pattern[1], 3), np.float32)
    obj[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2) * square_mm
    # Every fifth view is excluded from intrinsic fitting and tests generalisation.
    train = [i for i in range(len(corners)) if i % 5 != 0]
    holdout = [i for i in range(len(corners)) if i % 5 == 0]
    rms, K, d, _, _ = cv2.calibrateCamera([obj] * len(train), [corners[i] for i in train], size, None, None)
    errors = []
    for i in holdout:
        ok, rotation, translation = cv2.solvePnP(obj, corners[i], K, d)
        if not ok:
            raise ValueError("Pose fit failed on a held-out board view")
        projected, _ = cv2.projectPoints(obj, rotation, translation, K, d)
        errors.append(float(np.sqrt(np.mean(np.sum((projected-corners[i])**2, axis=2)))))
    validation_rms = float(np.sqrt(np.mean(np.square(errors))))
    if not np.isfinite(validation_rms) or max(rms, validation_rms) > max_error:
        raise ValueError(f"Calibration rejected: training {rms:.3f}px, held-out {validation_rms:.3f}px; "
                         f"target <= {max_error}px. Use a flat, sharp board and more varied tilts.")
    if not (K[0, 0] > 0 and K[1, 1] > 0 and 0 < K[0, 2] < size[0] and 0 < K[1, 2] < size[1]):
        raise ValueError("Calibration produced implausible intrinsics")
    # Refit on all observations only after the held-out check has passed.
    final_rms, K, d, _, _ = cv2.calibrateCamera([obj] * len(corners), corners, size, None, None)
    output.mkdir(parents=True, exist_ok=True)
    metadata = {"camera": "Raspberry Pi Camera v1.3 / OV5647", "validated": True,
                "width": size[0], "height": size[1], "fps": 20, "K": K.tolist(),
                "distortion": d.ravel().tolist(), "square_mm": square_mm, "pattern": list(pattern),
                "views": len(corners), "training_rms_px": rms, "held_out_rms_px": validation_rms,
                "final_rms_px": final_rms, "images": images}
    (output / "intrinsics.json").write_text(json.dumps(metadata, indent=2) + "\n")
    (output / "camera.yaml").write_text(settings_text(K, d, *size))
    print(f"Saved calibration: {output}/camera.yaml; held-out error {validation_rms:.3f}px")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--folder", type=Path, default=Path("calibration/images"))
    parser.add_argument("--output", type=Path, default=Path("calibration"))
    parser.add_argument("--square-mm", type=float, default=20)
    parser.add_argument("--cols", type=int, default=9)
    parser.add_argument("--rows", type=int, default=6)
    parser.add_argument("--max-error", type=float, default=0.8)
    parser.add_argument("--solve-only", action="store_true")
    args = parser.parse_args()
    if args.square_mm <= 0:
        parser.error("--square-mm must be positive")
    pattern = (args.cols, args.rows)
    if not args.solve_only:
        args.folder.mkdir(parents=True, exist_ok=True)
        prior, last_saved = [], 0.0
        with PiStream(port=args.port) as stream:
            sequence = -1
            while True:
                sequence, stamp, image, jpeg, _ = stream.read(after=sequence)
                corners = detect(image, pattern)
                display = image.copy()
                if corners is not None:
                    cv2.drawChessboardCorners(display, pattern, corners, True)
                    desc = descriptor(corners, (image.shape[1], image.shape[0]))
                    diverse = not prior or min(np.linalg.norm(desc-p) for p in prior) > 0.09
                    if diverse and time.monotonic()-last_saved > 0.8:
                        (args.folder / f"view_{stamp}.jpg").write_bytes(jpeg)
                        prior.append(desc)
                        last_saved = time.monotonic()
                cv2.putText(display, f"Views: {len(prior)} | move/tilt board | C: solve | Q: exit", (8, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv2.imshow("Camera calibration", display)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    cv2.destroyAllWindows()
                    return
                if key == ord("c"):
                    break
        cv2.destroyAllWindows()
    solve(args.folder, args.output, pattern, args.square_mm, args.max_error)


if __name__ == "__main__":
    main()
