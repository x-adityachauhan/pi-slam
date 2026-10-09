# Third-party software and weights

This repository integrates upstream SLAM systems; it does not claim ownership
of their source code, algorithms, or pretrained weights. No new project-wide
license has been selected for the original integration code.

| Component | Pinned revision | Upstream license |
| --- | --- | --- |
| [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) | `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4` | GPL-3.0; see [license](third_party/ORB_SLAM3_LICENSE.txt) |
| [Pangolin](https://github.com/stevenlovegrove/Pangolin) | `aff6883c83f3fd7e8268a9715e84266c42e2efe3` (v0.8) | MIT; see [license](third_party/Pangolin_LICENSE.txt) |
| [DPVO / DPV-SLAM](https://github.com/princeton-vl/DPVO) | `0ac95b656d1fda91c271d2a106460d19ad966fc7` | MIT; see [license](gpu/weights/LICENSE) |

The CPU bootstrap downloads the pinned upstream sources and applies the
patches in `verification/`. Respect the upstream licenses when using or
redistributing them, including the GPL obligations for ORB-SLAM3 modifications
and linked binaries. Dependencies and bundled upstream subcomponents retain
their own licenses.

The included pretrained DPVO checkpoint is documented in
[gpu/weights/README.md](gpu/weights/README.md), including its source and SHA-256.
The GPU Docker build fetches DPVO at the revision above and builds its CUDA
extensions. The portable releases contain inference code and weights, not
Linux environments or precompiled SLAM libraries.
