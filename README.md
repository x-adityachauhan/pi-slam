# Pi camera SLAM on two laptops

**Current target: manual RC-car testing with 3D surface reconstruction.**
Use [rc/README.md](rc/README.md) and `bash rc/run.sh` on this laptop.
The new RTX portable package is `pi-slam-rc-rtx5060.tar.gz`.
Drone integration is future work in [drone/PLAN.md](drone/PLAN.md), not a
flight-ready feature. The original commands below remain sparse-map tools.

## Fresh clone and releases

```bash
git clone https://github.com/x-adityachauhan/pi-slam.git
cd pi-slam
bash bootstrap-cpu.sh
```

The CPU bootstrap supports Linux x86_64. Install `git`, `curl`, `tar`, `bzip2`
and a working C++ compiler first, and allow several GB for dependencies and
builds. It creates an isolated environment from `environment-explicit.txt`,
downloads pinned ORB-SLAM3/Pangolin sources, applies the checked-in patches,
and builds the CPU runner. The GPU build uses `bash gpu/build.sh`; it has not
yet been verified on the second laptop.

[GitHub releases](https://github.com/x-adityachauhan/pi-slam/releases) contain
the portable laptop archives and checksums. Git contains source, calibration
parameters, the pretrained GPU checkpoint, dependency manifest, patches and
verification reports. Raw camera images, recordings, generated run outputs,
local environments and compiled binaries stay local. Verification reports
describe local runs; rerun the tests to generate their artifacts elsewhere.
See [third-party notices](THIRD_PARTY.md) before redistribution.

The included calibration applies only to the original camera and capture
mode. Recalibrate a different camera. Change the Pi target with
`PI_TARGET=ras@YOUR_PI_IP bash start-camera.sh`; SSH passwords are not stored.
Examples below use the original `/home/kali/pi-slam` path; replace it with
your clone's path if needed. Launchers locate their files relative to themselves.

The Raspberry Pi 4 captures its Camera v1.3 (OV5647). The laptop receives
640×480 Motion-JPEG frames at 20 FPS over an SSH tunnel and runs SLAM locally.
Capture timestamps come from the Pi's V4L2 driver, not Wi-Fi arrival times.
Receivers process the newest frame to avoid a growing video backlog.

| Laptop | Backend | Live output |
| --- | --- | --- |
| This Kali laptop, i5-13420H, 16 GB RAM | ORB-SLAM3, CPU | Camera tracking, Pangolin 3D map |
| Ultra 9 275HX, RTX 5060, 16 GB RAM | DPV-SLAM, CUDA | Camera and sparse 3D map in a local browser |

These are integrations of existing SLAM systems and pretrained inference
weights, not newly trained models. Neither a feature map nor a single-camera
trajectory provides guaranteed accuracy or metric distances. Moving objects,
low light, blur, textureless walls, rapid rotations, and rolling-shutter motion
can degrade tracking. Move the camera through a static, well-lit scene with
both near and far textured objects; translation is needed to initialize depth.

## First laptop: camera and calibration

This camera is already calibrated at 640×480 / 20 FPS: 59 views, held-out
reprojection RMS 0.271 pixels. Keep the saved files unless the capture mode or
optics change. The printing and collection steps below are for recalibration.

In terminal 1:

```bash
cd /home/kali/pi-slam
bash start-camera.sh
```

Enter the Pi SSH password when prompted. Leave this terminal running. Ctrl+C
stops the camera process and tunnel. The helper was deployed to
`/home/ras/pi-slam/camera_server.py`; existing Pi camera scripts were preserved.
For redeployment use `bash start-camera.sh --deploy`.

In terminal 2, preview the camera:

```bash
cd /home/kali/pi-slam
bash run-python.sh preview.py
```

Press Q to close the preview before starting calibration or SLAM. The Pi
supports one active receiver at a time. Do not start both laptops' camera
helpers simultaneously.

Print `calibration_board.svg` on A4 at **100% / actual size**, without “fit to
page”. Mount the paper on something rigid and flat. Measure a square: the
intended size is 20 mm. If the actual size differs, pass `--square-mm SIZE`.
The board has 9×6 inner corners (10×7 squares).

```bash
bash run-python.sh calibrate.py --square-mm 20
```

Show the whole board, with a white margin visible around it. Slowly move it
across the image, closer and farther, and tilt it around both axes. The tool
automatically collects visibly different views. Collect at least 25 views;
30–40 is preferable. Press C to solve, or Q to exit. Board centers must cover
at least four of the image's nine grid regions. One in five views is held out
from fitting and validates the calibration. The default error target is
0.8 pixels; a failed validation saves no approved calibration.

Successful output:

- `calibration/camera.yaml`: settings for ORB-SLAM3.
- `calibration/intrinsics.json`: the same camera model for the CUDA backend.
- `calibration/images/`: the observations used for calibration.

The board's square size does **not** automatically establish scale during
subsequent SLAM. The map still uses an arbitrary scale. Recalibrate if you
change the lens/focus, resolution, crop, rotation, or camera capture mode.

## First laptop: run SLAM

With terminal 1 still streaming:

```bash
cd /home/kali/pi-slam
bash run-slam.sh
```

The launcher requires a validated calibration and creates a new directory in
`runs/` for each session. The camera window's **Q** key, map viewer's **Stop**
control, or Ctrl+C in the
launcher saves the session. The console reports tracking state: 1 means not
initialized, 2 means tracking, and 3 means recently lost. The camera needs
sideways translation with visible detail to initialize.

Exports include:

- `atlas.osa`: ORB-SLAM3's serialized atlas.
- `map_N.ply`: reliable sparse landmarks for each independent map.
- `map_N_keyframes.tum`: optimized keyframe poses for that map.
- `poses_live.tum`: online poses before subsequent loop corrections.
- `stats.json`: processed/skipped frame counts and tracking timing.

Different map IDs may have independent coordinate systems. Do not concatenate
their clouds as if they share a common origin or scale.

For a bounded, headless session:

```bash
bash run-slam.sh --headless --seconds 60
```

The build uses isolated dependencies in `env/`, leaving system packages
unchanged. To rebuild the existing checked-out sources:

```bash
bash build-cpu.sh
```

## Second laptop: RTX 5060 package

Use the second laptop's **Kali installation**. Native Windows is not the target
of this package. Linux requires a working NVIDIA driver, Docker, and NVIDIA
Container Toolkit. `nvidia-smi` must recognize the RTX 5060. Follow the
[NVIDIA Container Toolkit installation guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
to enable Docker GPU access. Reserve about 30 GB of disk for the development
container, downloaded layers, and build cache. Compile concurrency is limited
to two jobs for a machine with 16 GB RAM.

Copy `pi-slam-rtx5060.tar.gz` to the second laptop, then:

```bash
mkdir -p "$HOME/pi-slam-rtx5060"
tar -xzf pi-slam-rtx5060.tar.gz -C "$HOME/pi-slam-rtx5060"
cd "$HOME/pi-slam-rtx5060"
bash gpu/build.sh
bash gpu/run.sh --doctor
```

Docker access may require `sudo` depending on that laptop's setup. The image
uses PyTorch 2.7.1/CUDA 12.8 and builds native CUDA extensions for `sm_120`,
including the RTX 5060. The source adapter updates old tensor dispatch and
atomic APIs without modifying the learned model or its inference algorithm.
The package includes the upstream pretrained `dpvo.pth` checkpoint; no training
dataset is downloaded. `--doctor` checks the real GPU and executes model and
CUDA operations before you attempt a camera session.

The archive already includes this camera's **validated**
`calibration/intrinsics.json`. Replace it if you recalibrate before using the
second laptop. Calibration is reusable because both pipelines use the same
Pi camera and capture mode.

Stop terminal 1 on the first laptop before starting terminal 1 on the second:

```bash
cd "$HOME/pi-slam-rtx5060"
bash start-camera.sh
```

Then, in terminal 2:

```bash
cd "$HOME/pi-slam-rtx5060"
bash gpu/run.sh
```

Open **http://127.0.0.1:8090** in a browser. Drag to rotate the map, scroll to
zoom, and use Fit map to recenter. Ctrl+C in terminal 2 saves the final map and
trajectory. The CUDA frontend undistorts images with the actual camera model
before inference and preserves original capture times.

The GPU profile uses 96 patches/frame, mixed precision, a 2048-keyframe
capacity, and loop closure with a bounded 512-keyframe patch history.
The optional classical long-range image-retrieval backend is not installed.
At capacity, the run stops and saves rather than overwriting map memory.
Increasing `--buffer` or the loop history requires additional VRAM; actual
memory use and speed must be measured on the second laptop.

Outputs are `trajectory.tum`, `map.ply` (colored sparse points), and
`stats.json` in a new `runs/dpv-*` directory. If motion is insufficient to
initialize, the tool reports that condition instead of declaring an empty
map successful.

**The RTX image has not been compiled or run on the second laptop yet.** It is
a prepared build package; its CUDA compilation, runtime speed, and memory
usage need verification on that hardware. The first laptop has no CUDA GPU.

## Verification and sources

The transport's real-camera check received 100 frames at approximately
20 FPS with no sequence gaps. `verification/live_camera.jpg` is that capture's
first frame; it was dark and had little scene detail. Calibration and tracking
need a brighter, textured view.

```bash
bash run-python.sh -m unittest discover -s tests -v
```

Tests cover fragmented network data, invalid timestamps, V4L2 structure sizes,
calibration rejection, recovery of known synthetic intrinsics, and the local
map viewer endpoints. Synthetic camera tests are not measurements of physical
camera accuracy. See `verification/REPORT.md` for the final integration results.

The final real-camera GUI test processed 1,199 frames, tracked 414, and saved
2,734 reliable points across **three separate maps**. Mean tracking time was
19.3 ms/frame. Tracking restarted twice, so this proves working capture,
tracking, display, and saving—not guaranteed continuous tracking or measured
physical accuracy. Each map must remain in its own coordinate system.

Primary upstream sources:

- [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3), revision `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4` (GPLv3).
- [DPV-SLAM / DPVO](https://github.com/princeton-vl/DPVO), revision `0ac95b656d1fda91c271d2a106460d19ad966fc7` (MIT).
- [Pangolin](https://github.com/stevenlovegrove/Pangolin), v0.8 (MIT).
- [OpenCV camera calibration](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html).
- [PyTorch 2.7 Blackwell and CUDA 12.8 support](https://pytorch.org/blog/pytorch-2-7/).
- [V4L2 capture timestamps](https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/buffer.html).
