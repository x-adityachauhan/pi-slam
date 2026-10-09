# RC-car mapping verification — 2026-10-09

Current scope: laptop SLAM and 3D mapping with the Pi camera. No car steering,
drone commands, flight-controller output, metric VIO or flight readiness is
implemented. The earlier drone plan is retained as future work.

## Implemented and checked

- Existing ORB-SLAM3 CPU tracker rebuilt with original JPEG recording,
  capture-index CSV, optional atomic pose/status output and frame-count limit.
- Camera-only research health gate always refuses metric/flight use; explicit
  checks cover stale/lost/discontinuous estimates, resets and recovery.
- Calibrated multiview stereo from final optimized per-map poses: rectification,
  positive/negative disparity, left/right consistency, texture/speckle rejection,
  camera-to-world transform, bounded voxel fusion and cross-pair support.
- Reconstructed voxel must have support from at least two different image pairs;
  multiple samples in one pair count once. Unseen space is not classified free.
- Browser viewer loads one independent map, preserves RGB colors, bounds display
  sampling and shows the saved camera path. No independent maps are merged.
- New CPU launcher rc/run.sh tracks/records first and reconstructs after Q/Stop.
- RTX frontend source records matching images and reconstructs after termination;
  CUDA image remains uncompiled/unverified on the actual second laptop.
- start-camera.sh now allocates a remote PTY so Ctrl+C reaches the Pi helper.

## Automated tests

19 Python tests passed in 4.164 seconds. Logs: rc-python-tests.log.
They include the original transport/calibration/viewer tests and additional
known-depth planes, reversed stereo baseline, camera-to-world rotation,
pure-rotation/textureless-image rejection, pair selection, safe recorded paths,
voxel capacity, independent pair support and pose watchdog cases.
Python compilation, shell syntax and browser JavaScript syntax checks passed.

Known-plane tests use 320×240 images, focal length 240, baseline 0.2 and true
depth 3.0 synthetic units. Median reconstructed plane position is checked
within 0.025 units for both baseline signs and rotated world frame. These are
unit tests of geometry, not evidence of centimetre physical accuracy.

## End-to-end synthetic integration

Fixture: tests/synthetic_stream.py, 400-frame procedural trajectory; run lasted
12 seconds with original capture timestamps and actual ORB-SLAM3 processing.

- Processed: 226 frames; tracked: 221; skipped: 14.
- Mean tracking call time: 46.55 ms/frame on this CPU (not full pipeline latency).
- Sparse map: 1,984 reliable landmarks and 26 optimized keyframes.
- Recorded JPEGs/index and native pose events verified.
- Dense reconstruction: 12 attempted pairs, 11 accepted; one vertical epipolar
  pair rejected. 64,045 candidate voxels, 19,495 supported colored surface points.
- Voxel size: 0.00516815 arbitrary map units; no capacity drops.
- Stereo reconstruction and export: 2.35 seconds for this fixture.

Artifacts: rc-synthetic/run; logs rc-integration.log and rc-dense.log.

Surface comparison against the fixture's known rectangle surfaces uses Sim(3)
alignment explicitly removing monocular scale. Median nearest-surface error:
0.02286 scene units; p90: 0.07131; 95.61% of points within 0.1 scene units.
This measures neither completeness nor collision safety, and does not establish
physical units or real-world error. See rc-surface-error.json and
tests/evaluate_rc_scene.py. Outliers and holes remain possible.

An initial one-pair-supported output contained 108,062 points but appreciably
more errors. It was replaced by the cross-pair-supported reconstruction above.
Point count alone is not an accuracy score.

## Current physical camera check

Run: ../runs/rc-20261009-104940-MMObAZ.

- Recorded 240 valid JPEGs and matching chronological pose events in 12 seconds.
- Capture rate: 20.0056 FPS; zero skipped frames.
- Every status event remains nonmetric with vehicle output disabled.
- Tracked frames: zero. The inspected last image is almost black.
- Atlas saved, but no map initialized; dense reconstruction correctly reported
  missing optimized map poses instead of declaring an empty map successful.

This check verifies transport/recording, not real RC-car mapping. The earlier
physical ORB test (REPORT.md) did track/export maps but restarted twice. A new
well-lit, translated RC-car recording and independent references are needed to
evaluate physical mapping performance. The new dense stage has not yet produced
a verified physical-camera map.

## Remaining validation

- Actual manually driven RC-car capture in a static textured scene.
- Real repeatability, trajectory ground truth, surface error, coverage and
  processing/transport latency distributions.
- Second laptop GPU build/runtime, resource use and accuracy.
- IMU/depth/stereo hardware and calibration for future metric/drone work.
- Autopilot integration, time/frame/uncertainty validation, SITL and flight tests.

No newly trained network, guaranteed accuracy, complete mesh, obstacle-free path,
metric monocular scale or flight qualification is claimed.

Implementation uses [OpenCV stereo geometry](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)
and the pinned tracking backends documented in ../README.md.
