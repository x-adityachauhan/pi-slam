# Step 4 — experimental asynchronous CPU depth

MiDaS v2.1 small OpenVINO model (256 square input), CPU-only two-thread
inference, cached on at most 16 keyframes. Model source and SHA256 are in
rc/setup-depth.sh and rc/depth-model.sha256. Local binaries are ignored.
Preprocessing follows upstream RGB/ImageNet normalization and square resize.
OpenVINO 2026.4.1, scikit-image 0.26.0. Single zero-input inference: 119 ms;
this is a smoke measurement, not a warmed real-scene benchmark.

Inference is on a separate worker thread in a lower-priority process, not the
tracking thread. Latest snapshots replace queued work. Robust affine inverse-
depth alignment requires 30 anchors, image coverage, positive scale, >=65%
inliers and <=15% held-out median relative error. These heuristic gates are not
a calibrated uncertainty bound or proof of physical accuracy.

An 80^3 local TSDF integrates accepted predicted depth with three-voxel
truncation. Faces touching insufficiently supported voxels are rejected. The
volume is rebuilt from current keyframe poses on update, including map changes;
it does not accumulate a permanent global room mesh. Runtime cache is bounded.
Map coordinates and scale remain arbitrary. Range bins project predicted mesh
vertices on map X-Z; this is not LiDAR or gravity-aligned obstacle sensing.

Same recorded 655-frame replay with worker enabled:

| Metric | Live display, no depth | Live display + depth |
| --- | ---: | ---: |
| Delivered FPS | 12.25 | 12.25 |
| Tracked % | 90.53 | 89.77 |
| Mean keypoints | 1673 | 1673 |
| Map count | 1 | 1 |
| Tracking-call mean ms | 26.98 | 27.06 |

Single trials, asynchronous nondeterminism: do not attribute the entire tracking
difference to depth. The input itself contains 414 missing source frames.

Worker updates took approximately 0.14–0.94 seconds. Intermediate volumes
produced up to 1,655 triangles from six accepted keyframes. Many predictions
were rejected; final snapshot accepted only one keyframe and has zero triangles.
This is NOT a stable dense room reconstruction. No thresholds were weakened
to hide this failure. Physical after-test and real accuracy remain unverified.

34 Python tests passed; one optional native test skipped. New tests cover
inverse-depth scale/shift recovery, degenerate-anchor rejection and TSDF plane
integration requiring multiple observations. Native build and JS syntax passed.
The preceding live browser check verified overlays and sparse geometry; dense
browser/range visual acceptance and a complete-room demonstration are pending.

No changes under GPU, shared viewer, original camera launcher or original camera
server relative to the approved baseline commit. Original recordings retained.
