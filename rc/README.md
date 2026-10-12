# RC-car camera-only 3D SLAM

## Updated CPU live workflow

Use `bash rc/start-camera.sh --deploy` for the opt-in CPU camera profile
(10 ms requested exposure, auto ISO, JPEG quality 75). Keep the scene bright.
Run `bash rc/run.sh` in another terminal. It prints the unique run directory
and a `rc/live_view.py` command; run that command for the CPU browser on 8093.
The camera has green mapped-feature and orange other-feature overlays, state
and delivered FPS. These are feature detections, not semantic object labels.
The CPU browser includes live trajectory, sparse landmarks and an X-Z view.
X-Z is not gravity-aligned without a mounting assumption or external sensor.

Optional learned depth (experimental, CPU only):

```bash
bash rc/setup-depth.sh
bash run-python.sh rc/depth_worker.py runs/YOUR_CURRENT_RUN
```

Enable **Predicted TSDF** in the CPU browser. Inference runs in a worker thread
in a separate, lower-priority process with two inference threads. The latest
16 keyframes are the bounded input set; superseded snapshots are skipped.
Inverse-depth scale and offset are fitted to SLAM anchors with held-out checks.
Rejected predictions are not fused. An 80-cubed local TSDF is rebuilt from
current poses each update; it is not a persistent whole-building mesh. The
range panel bins predicted surfaces in map X-Z, not actual LiDAR measurements.
Scale is arbitrary. Missing bins/voxels are unknown, never safe free space.
Older dense publications are hidden after five seconds or map changes.

Source: [MiDaS](https://github.com/isl-org/MiDaS), MIT-licensed model repository.
Model binaries remain local and ignored by Git. No GPU/DPV entrypoint or viewer
was changed. See verification/CPU_LIVE_DIAGNOSIS.md and CPU_STEP2/3/4.md.

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
   poses, pose-checked stereo pairs and independent-view depth validation.
   Each retained depth pixel must agree with two other camera views before
   vectorized voxel fusion. Agreement is tested before voxel downsampling.
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

Transient receive stalls are tolerated for up to 15 seconds without discarding
partially received frame bytes. The Pi allows 10 seconds for a blocked send.
A closed connection still stops capture; this does not automatically reconnect
or bridge a tracking gap. The runner saves a summary and any retained map,
records `capture_error.txt`, and returns status 3 on stream failure. The launcher
then reconstructs a partial map if tracking had initialized. A run that never
initialized is explicitly reported as no result (launcher status 4), not shown
as a successful new map. Restart `start-camera.sh` if the tunnel itself failed.

The camera overlay and pose events include the native feature count. Many
features alone do not establish depth: matched features must also pass the
two-view geometry tests. Initialization messages are printed every ten capture
seconds, and live progress is flushed immediately instead of being buffered.

The launcher prints the unique run directory. It saves:

- atlas.osa and separate map_N.ply / map_N_keyframes.tum.
- frames.csv and original timestamped JPEG images in frames/.
- pose_events.jsonl and atomic latest_pose.json for diagnostic monitoring.
- A copy of camera intrinsics used for the run.
- dense_v2/map_N_dense.ply: colored reconstructed surfaces for each map.
- dense_v2/map_N_mesh.ply: triangles connecting measured, consistent neighboring
  depth pixels; holes and depth discontinuities are preserved.
- dense_v2/stats.json: accepted/rejected pairs, point count, voxel size, capacity
  drops and reconstruction errors. Empty/failed reconstructions are reported.

Earlier `dense/` files remain available for comparison. The new report also
includes descriptor-match/epipolar checks, depth support, cache reads and a
representative optimized camera view. `--method pairwise` explicitly selects
the earlier method and writes its original `dense/` destination.

There is no complete watertight mesh and no declaration that unseen space is
safe. Never merge independently reset maps without estimating their alignment.
Reconstruction handles horizontal and vertical stereo baselines, including
negative disparity. Low-overlap pairs, inconsistent matches and degenerate
geometry are rejected. The default CPU budget is two OpenCV threads, up to
40 candidate pairs per map (maximum configurable budget: 120), and at most 80
optimized views. Two stereo workers run concurrently with bounded queued data.
Source images and features are cached; final voxel fusion is vectorized.

View one reconstructed map:

```bash
bash run-python.sh rc/view_map.py /home/kali/pi-slam/runs/rc-YOUR-RUN
```

Open http://127.0.0.1:8090 if a browser does not open automatically. The viewer
chooses `dense_v2/` when present, then falls back to `dense/`. Use
`--reconstruction dense` to inspect the old result. Its camera view aligns
points with an undistorted optimized reference image. Drag to orbit, scroll
to zoom, or use Camera view / Orbit controls. The map selector keeps independent
maps separate. GPU depth testing handles occlusion; point size is adjustable.
Up to 200,000 points are displayed; Download PLY retains the full mesh when
available, otherwise the point cloud. The Surfaces checkbox toggles triangles.
Saved maps load once instead of being transferred twice per second.

To repeat reconstruction without recapturing:

```bash
bash run-python.sh rc/dense_reconstruct.py /home/kali/pi-slam/runs/rc-YOUR-RUN \
  --calibration /home/kali/pi-slam/runs/rc-YOUR-RUN/intrinsics.json --max-pairs 80
```

Use `--max-pairs 24` to reduce postprocessing work at the cost of coverage;
`--threads 2` controls OpenCV's thread budget. `--min-support 2` means two other
views agreeing with each reference depth (three cameras in total). Lowering
this below two is rejected by the revised method. `--output-name NAME` allows
comparisons in separate subdirectories. Unobserved areas remain holes.

The live camera overlay reports low light and soft images. Move slowly sideways
with overlap, keeping textured objects at several depths visible. Check lens
cleanliness/focus if a stationary, well-lit view still looks soft. Image quality
is also recorded in pose events. Better capture is necessary for complete room
coverage; the existing recording has tracking gaps.

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
