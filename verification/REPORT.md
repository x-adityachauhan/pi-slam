# Verification report — 2026-10-09

## Scope and result

The first laptop's CPU pipeline is built and exercised end-to-end with the
physical Pi camera, including both GUI windows and saved atlas/map exports.
The second laptop's CUDA pipeline is a prepared source/build package, not a
verified GPU build. That laptop was unavailable and this laptop has no CUDA GPU.

These integrate ORB-SLAM3 and pretrained DPV-SLAM; they are not newly trained
models. Output is sparse geometry with arbitrary monocular scale. No physical
ground-truth trajectory or distance measurements were available.

## Camera and calibration

- Raspberry Pi 4, Ubuntu 22.04 arm64; ribbon-connected OV5647 / Camera v1.3.
- Legacy camera detection: supported=1, detected=1; V4L2 camera0 /dev/video0.
- Hardware JPEG, 640×480, requested 20 FPS, quality 90, loopback-only listener.
- SSH-encrypted transport preserves driver monotonic capture timestamps.
- Initial physical transport check: 100 frames, approximately 20.006 FPS,
  zero sequence gaps. The first preview was dark and blurry.
- Calibration: 59 checkerboard views, 9×6 inner corners, 20 mm squares.
- Training reprojection RMS: 0.3373 px; held-out RMS: 0.2708 px;
  final all-view RMS: 0.3244 px. Held-out views were excluded during validation.
- Validated files: ../calibration/intrinsics.json and ../calibration/camera.yaml.

Reprojection error is not SLAM position error. Calibration square size does
not establish scale in subsequent monocular SLAM sessions.

## First laptop

Intel i5-13420H, 16 GB RAM, Kali Linux. Dependencies are isolated in ../env;
no broad system upgrade was performed. OpenCV 4.10, Eigen 3.4, Pangolin 0.8,
ORB-SLAM3 revision 4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4.

### Automated checks

All seven Python tests passed in 4.20 seconds on the final run. They cover
fragmented transport, preserved timestamps/pixels, rejection of duplicate
timestamps, native V4L2 ABI layout, missing calibration observations,
synthetic calibration recovery, ORB setting numeric types, and viewer HTTP
endpoints. Shell syntax checks also passed.

### Procedural scene integration

The deterministic 3D billboard fixture uses known intrinsics and camera motion.
Results in synthetic/run/stats.json: 210 processed frames, 207 tracked,
10 skipped, 1,835 exported reliable points and 29 keyframes; mean tracking
43.24 ms/frame. This is a deliberately feature-rich scene, unlike a blank wall.

Similarity-aligned position RMSE: 0.004187 synthetic units, or 0.349% of the
observed trajectory diameter. See synthetic/position-error.json. This smoke
test passes its 4% relative threshold but does not measure physical accuracy.
The alignment explicitly removes monocular scale and coordinate-frame ambiguity.

Reproduce the error calculation:

```bash
bash run-python.sh tests/evaluate_synthetic.py verification/synthetic/run/poses_live.tum
```

### Physical camera GUI and saving

Final 60-second GUI test: ../runs/orb-20261009-102136-tEIrgp.
Process exited successfully, displayed camera and Pangolin map windows,
and saved a 15,647,845-byte atlas.osa.

- Processed frames: 1,199; tracked frames: 414; skipped frames: 1.
- Mean tracking call time: 19.3021 ms/frame (not total GPU/CPU pipeline latency).
- Map 0: 1,996 reliable points, 55 keyframes.
- Map 1: 314 reliable points, 16 keyframes.
- Map 2: 424 reliable points, 12 keyframes.
- Total reliable points: 2,734 across three independent maps.

Tracking was lost and restarted twice. The successful export proves capture,
tracking and saving function, not uninterrupted tracking or high physical
accuracy. Preserve the separate coordinate systems; do not merge PLY files
without an estimated alignment. See live-final.log and that run's stats.json.

### Saving regression and upstream adaptations

An earlier live run crashed during atlas saving. A subsequent debug run saved
normally, so no stack trace established the exact cause of that earlier crash.
Inspection found unsafe iterator mutation in upstream observation pruning.
We patched Map/MapPoint PreSave to iterate snapshots, handle an empty
reference-keyframe observation set, and omit points made bad during pruning.
Atlas PreSave now clears its backup vector before repeated saves.

The native regression test loads a saved physical atlas read-only, stops all
workers, then creates stale-map and absent-keyframe observations in the
in-memory copy. Two consecutive PreSave calls passed: 1,311 points pruned to
525 points, with 2,270 valid observations remaining. Original disk files are
not overwritten. See save-regression.log and ../tests/save_regression.cc.

Other adaptations: C++17/modern dependencies, corrected loop BA counter type,
X11 Pangolin backend, OpenCV camera GUI confined to the main thread, and
waiting for mapping/loop/viewer workers to stop before serialization/export.
All source changes are retained in orb-kali.patch and pangolin-kali.patch.

Reproduce the saving regression against the preserved fixture:

```bash
cd /home/kali/pi-slam/runs/orb-20261009-101556-ZdnSCb
LD_LIBRARY_PATH=/home/kali/pi-slam/env/lib:/home/kali/pi-slam/vendor/ORB_SLAM3/lib:/home/kali/pi-slam/vendor/ORB_SLAM3/Thirdparty/DBoW2/lib:/home/kali/pi-slam/vendor/ORB_SLAM3/Thirdparty/g2o/lib \
  /home/kali/pi-slam/build/pi_slam_save_test \
  /home/kali/pi-slam/vendor/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  /home/kali/pi-slam/verification/save-test.yaml
```

## Second laptop: prepared, not hardware-verified

Target: Kali Linux on Ultra 9 275HX, RTX 5060, 16 GB system RAM.
Windows-native operation is not packaged. DPVO revision
0ac95b656d1fda91c271d2a106460d19ad966fc7 with loop closure, 96 patches/frame,
mixed precision, 2,048-keyframe capacity, 512-keyframe patch history, and a
local browser viewer. Optional classical long-range retrieval is disabled.

The Dockerfile pins PyTorch 2.7.1 / CUDA 12.8 and builds extensions for sm_120
with two compilation jobs. PyTorch documents Blackwell support in its
[2.7 release](https://pytorch.org/blog/pytorch-2-7/). Mechanical source
adaptations update dispatch, atomics and checkpoint loading; see dpvo-cu128.patch.
Pretrained dpvo.pth is included with its upstream license/provenance and SHA-256:
30d02dc2b88a321cf99aad8e4ea1152a44d791b5b65bf95ad036922819c0ff12.

Verified here: frontend Python syntax, source-adapter inspection, shell syntax,
browser endpoint tests, checkpoint checksum, and archive contents. Not verified:
Docker image compilation, CUDA extension/runtime compatibility, real GPU memory
use, throughput, continuous tracking, loop closure accuracy, or Windows support.
Run gpu/build.sh and gpu/run.sh --doctor on the actual target before live use.
CUDA errors must be resolved there rather than interpreting this package as a
completed, hardware-tested deployment.

The portable archive includes the existing validated camera calibration;
calibration observations and large CPU dependencies/test artifacts stay on
the first laptop. Both laptops must use the same capture mode. Only one camera
receiver/session should run at a time.
