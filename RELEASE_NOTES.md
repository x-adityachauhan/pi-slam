# RC-car 3D SLAM prototype

Pi Camera v1.3 capture, calibrated CPU ORB-SLAM3 integration, optional RTX 5060
DPV-SLAM integration, recorded-frame temporal stereo reconstruction, saved 3D
viewer, and verification tests/reports.

- `pi-slam-rc-rtx5060.tar.gz`: current portable package with RC-car reconstruction.
- `pi-slam-rtx5060.tar.gz`: earlier sparse-SLAM-only GPU package.
- `rc-package.sha256` and `rtx-package.sha256`: archive integrity checks.

Use the current RC package for new tests. Linux x86_64 is the supported CPU
platform; the GPU package targets Kali/Linux on the second laptop. Native
Windows installation is not provided. Fresh-clone CPU instructions are in
README.md. GPU setup is in README.md and RC operation in rc/README.md.

CPU tests and synthetic reconstruction passed locally. The actual RTX 5060
Docker/CUDA build and live GPU tracking have not been tested because that
laptop was not available. Physical-camera dense reconstruction still needs
a successful, well-lit RC-car run; the latest live check did not initialize
tracking. See verification/REPORT.md and verification/RC_REPORT.md.

Single-camera reconstruction has arbitrary scale. This prototype does not
provide a metric obstacle map, RC-car control, autonomous navigation, or
flight-ready GPS-denied drone navigation. Future drone work is documented in
drone/PLAN.md. Raw camera photos, recordings, credentials, environments and
compiled build artifacts are not published.
