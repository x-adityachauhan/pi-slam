# GPS-denied drone SLAM — implementation plan

## Architecture

```text
Pi camera + synchronized calibrated IMU
             |
       local visual-inertial pose -------- health/time/frame checks
             |                                      |
       global SLAM map                         autopilot EKF input
             |                              (NOT enabled in this build)
aligned measured depth / calibrated stereo
             |
      3D occupied / free / unknown voxels
             |
      clearance-aware 3D route preview
```

The flight controller stabilizes the aircraft; SLAM does not replace its
attitude controller. Keep local odometry continuous. Loop closure changes the
global map transform, not an unannounced flight-control coordinate system.
Depth inferred by a neural network is not a measured collision boundary.

## Stage 1 — research foundation (implementation status)

- Preserve the calibrated Pi Camera v1.3 and existing CPU/GPU applications.
- Add a drone bench launcher with latest pose/status, capture times,
  receiver-to-result processing age, tracked-landmark count and map ID.
- Add a log-only supervisor that rejects nonmetric, stale, lost or discontinuous
  estimates. No vehicle messages, arming or autonomous setpoints are sent.
- Planned, deferred while focusing on RC-car tests: add an ORB-SLAM3
  monocular-inertial CSV replay executable. It will require
  camera/IMU calibration, chronological samples and bounded IMU gaps, and
  report whether inertial initialization actually succeeded.
- Planned: add metric-depth voxel integration and offline 3D route preview. Missing,
  unknown and invalid-depth space is not declared free.
- Planned: add deterministic simulated camera/IMU and depth/planning tests. Simulation
  verifies implementation behavior, not real-world flight accuracy.

## Stage 2 — requires your actual sensors

Identify flight controller/firmware, IMU and depth/stereo hardware. Obtain:

- Camera–IMU rigid transform and temporal offset, calibrated noise densities,
  bias random walks, axes and units. Camera intrinsics alone are insufficient.
- High-rate unfiltered gyro/accelerometer measurements on the same timebase as
  camera exposure. Telemetry attitude angles are not raw IMU measurements.
- Depth/stereo calibration and synchronization with pose, plus drone physical
  dimensions. Monocular depth/scale must not be silently relabeled as metres.
- Measured exposure/readout/transport delay and loss distributions. The current
  20 FPS rolling-shutter camera and wireless laptop link are bench equipment,
  not a validated flight-localization chain.

Engineering targets to evaluate, not promised performance: camera 30–60 FPS,
IMU around 200 Hz or higher, and sufficiently frequent external odometry for
the chosen firmware. Evaluate a global-shutter camera and synchronized stereo
or depth if rapid motion/obstacle avoidance is required. Do not buy a specific
sensor until weight, range, lighting, interface and compute constraints are known.

## Stage 3 — hardware-specific integration

Choose the estimator/backend using replay measurements. ORB-SLAM3's existing
inertial mode is the initial research backend; a continuous local VIO estimator
can be substituted without changing the map API. Benchmark on both laptops;
the RTX package is not hardware-verified. GPU speed does not supply missing IMU
measurements or remove timing/rolling-shutter errors.

Implement camera/IMU capture with a shared clock, calibrated body transform,
metric pose and velocity, genuine uncertainty, estimator reset counter and
health reporting. Feed the chosen controller's supported external odometry
interface only after frame/timing validation. Firmware parameters are not
changed automatically. Fail-safe behavior must be chosen for the real vehicle;
tracking loss must not be answered by blindly maintaining an autonomous command.

Move critical local estimation onto an onboard companion computer for flight,
or explicitly qualify the ground-laptop link and independent onboard fallback.
The laptop can continue dense mapping, logging and visualization. GPS denied
does not imply Wi-Fi availability.

## Stage 4 — verification ladder

1. Unit tests, recorded/simulated VIO and true-scale error evaluation.
2. Flight-stack SITL: latency/loss injection, estimator resets, frame transforms,
   unknown-space rejection and fallback transitions.
3. Props-off bench and handheld sensor trials against independent ground truth.
4. Controlled supervised trials only after the earlier evidence is accepted.

Measure SE(3)-aligned metric trajectory error (do not hide scale error with
Sim(3)), relative pose error, availability, p95/p99 latency, initialization,
reset frequency and obstacle-map precision/recall. Define acceptance limits
from vehicle speed, clearance and stopping distance—not a generic SLAM score.

## Primary references

- [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3): visual/inertial modes.
- [PX4 VIO](https://docs.px4.io/main/en/computer_vision/visual_inertial_odometry).
- [PX4 external estimation](https://docs.px4.io/main/en/ros/external_position_estimation): frames, timing and EKF integration.
- [MAVLink ODOMETRY](https://mavlink.io/en/messages/common.html#ODOMETRY): metres, covariance and reset counter.
- [ArduPilot non-GPS navigation](https://ardupilot.org/copter/docs/common-non-gps-navigation-landing-page.html).

No actual drone, calibrated IMU or depth sensor has been supplied for validation.

Current user priority is now the RC car: see ../rc/README.md. Drone hardware,
visual–inertial replay and flight integration are deferred, not claimed complete.
