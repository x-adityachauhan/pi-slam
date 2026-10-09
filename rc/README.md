# RC-car camera-only 3D SLAM

This is the current target. Drone navigation is deferred. The Pi captures
its calibrated camera; a laptop tracks motion, closes loops and saves a sparse
3D map. **After you stop**, calibrated multi-view stereo reconstructs richer,
colored scene surfaces using the final optimized camera poses.

No steering, motor, flight-controller or autonomous-flight commands exist.
The current sensor is monocular: positions, depth and voxel sizes use arbitrary
map units, not verified metres. This is a mapping prototype, not a collision-
avoidance or flight-localization system. Your intrinsic calibration remains valid
at 640×480 / 20 FPS as long as lens/crop/focus/mode do not change.

## Plan

1. Manual RC-car or handheld test: live tracking and repeatable recordings.
2. Richer 3D reconstruction: overlapping translated views, optimized per-map
   poses, disparity consistency filtering and bounded colored voxel fusion.
   By default, retain only voxels supported by at least two different image pairs;
   multiple pixels in one pair do not count as independent support.
3. Evaluate real tracking availability, map repeatability and independent
   reference measurements. Synthetic tests do not establish physical accuracy.
4. When ready, add synchronized IMU and measured stereo/depth for metric
   localization and occupied/free/unknown-space mapping. Only then address
   drone VIO, onboard compute, controller integration and flight validation.

The flight-oriented plan is retained in ../drone/PLAN.md as future work.

## This Kali laptop

Terminal 1:

```bash
cd /home/kali/pi-slam
bash start-camera.sh
```

Enter the Pi SSH password. Leave the stream running.

Terminal 2:

```bash
cd /home/kali/pi-slam
bash rc/run.sh
```

The camera window and live sparse 3D viewer open. Mount the camera rigidly on
the car, pointing towards well-lit, static, textured objects. Move slowly with
translation and overlapping views. For the first test, move past objects with
the camera pointed sideways or diagonally; forward-only motion/rotation and
blank walls are poor depth observations. Keep people/moving objects out of
the mapping view. Drive manually; this software does not control the car.

Press **Q** in the camera window, or use the map viewer's Stop control.
Ctrl+C also requests a graceful tracker stop; the launcher continues into
postprocessing. A cap of 3,000 processed frames limits recording storage;
typically about 150 seconds at 20 FPS. The actual capture rate and frame gaps
determine session duration. Expect roughly a few hundred MB/run depending on
JPEG size; watch free disk space. Save/export failures leave recorded files intact.

The launcher prints the unique run directory. It saves:

- atlas.osa and separate map_N.ply / map_N_keyframes.tum.
- frames.csv and original timestamped JPEG images in frames/.
- pose_events.jsonl and atomic latest_pose.json for diagnostic monitoring.
- A copy of camera intrinsics used for the run.
- dense/map_N_dense.ply: colored reconstructed surfaces for each map.
- dense/stats.json: accepted/rejected pairs, point count, voxel size, capacity
  drops and reconstruction errors. Empty/failed reconstructions are reported.

There is no complete watertight mesh and no declaration that unseen space is
safe. Never merge independently reset maps without estimating their alignment.
Reconstruction handles horizontal epipolar pairs, including negative disparity;
vertical/degenerate pairs and insufficient textured matches are rejected.

View one reconstructed map:

```bash
bash run-python.sh rc/view_map.py /home/kali/pi-slam/runs/rc-YOUR-RUN
```

Open http://127.0.0.1:8090 if a browser does not open automatically. Drag to
rotate, scroll to zoom, and Fit map to recenter. The largest successful map
is selected; use --map map_1 to choose another. The browser shows a bounded
sample for speed; the PLY file retains all fused points.

To repeat reconstruction without recapturing:

```bash
bash run-python.sh rc/dense_reconstruct.py /home/kali/pi-slam/runs/rc-YOUR-RUN \
  --calibration /home/kali/pi-slam/runs/rc-YOUR-RUN/intrinsics.json --max-pairs 80
```

Existing old SLAM runs have no recorded images and cannot be densified by this
tool. Capture a new run. Rerunning reconstruction overwrites that run's dense
outputs only, not its original images, poses or atlas.

Optional log-only pose watchdog:

```bash
bash run-python.sh -m drone.monitor /home/kali/pi-slam/runs/rc-YOUR-RUN
```

Camera-only pose is intentionally rejected by the metric research gate. The
landmark count is a heuristic count of nonbad matched landmarks, not calibrated
covariance or an inlier percentage. Receiver-to-result age is not end-to-end
camera latency because Pi and laptop clocks have not been synchronized.

## RTX 5060 laptop, Kali

Copy pi-slam-rc-rtx5060.tar.gz and follow ../README.md for NVIDIA driver,
Container Toolkit and Docker setup. Extract into a new directory. The package
includes calibration and pretrained weights. Its GPU image has **not** been
compiled/run on the actual target GPU. Native Windows is not packaged.

```bash
bash gpu/build.sh
bash gpu/run.sh --doctor
```

Stop the first laptop's camera stream before starting this one. Terminal 1:

```bash
bash start-camera.sh
```

Terminal 2:

```bash
bash rc/run-gpu.sh
```

Live sparse map: http://127.0.0.1:8090. Ctrl+C saves/optimizes the GPU session
and runs the same CPU multi-view surface reconstruction inside the container.
After completion, inspect the saved dense map:

```bash
bash gpu/view.sh runs/dpv-YOUR-RUN
```

Open http://127.0.0.1:8090. Browser viewing needs no GPU. Stereo matching in
this implementation is CPU OpenCV; RTX accelerates the DPV-SLAM tracker, not
this matching step. Backend performance/accuracy must be compared with recordings,
not assumed from GPU specs.

## Verification

```bash
bash run-python.sh -m unittest discover -s tests -v
```

See ../verification/RC_REPORT.md for actual test results and limitations.
Geometry uses [OpenCV calibrated stereo reconstruction](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html),
not a newly trained neural model. The tracking backends and licenses remain
those documented in ../README.md.
