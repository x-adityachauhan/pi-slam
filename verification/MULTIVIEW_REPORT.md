# Multiview reconstruction verification — 2026-10-12

## Scope

Improved the CPU post-processing reconstruction and saved-map viewer, while
keeping ORB-SLAM3 as the live tracking backend. This is not a new trained model,
metric navigation system or flight-ready implementation. The RTX laptop was
not available and its backend remains unverified on that hardware.

Reconstruction now checks feature matches against optimized camera poses,
rejects weak stereo geometry, supports vertical baselines, checks appearance
and left/right disparity, and checks depth and reprojection across other views.
It fuses accepted samples with bounded vectorized voxel processing. Mesh faces
connect supported neighboring depth pixels; they do not fill unknown regions.
Two workers and bounded caches limit CPU and memory use.

## Same-input synthetic comparison

Input: local `runs/synthetic-check-20261012/run`, identical saved camera poses
and images for both methods. Evaluation aligns the reconstruction to the known
procedural scene with a similarity transform, including scale. Distances below
are synthetic scene units, not metres or physical-camera accuracy.

| Measurement | Previous pairwise | New multiview |
| --- | ---: | ---: |
| Surface points | 62,622 | 56,985 |
| Median nearest-surface error | 0.03939 | 0.02606 |
| 90th-percentile error | 0.11694 | 0.07363 |
| Points within 0.1 scene units | 85.73% | 95.87% |

Median error decreased 33.9%. This measures proximity to known surfaces, not
scene completeness. The new mesh has 115,507 triangles; triangle-centroid
median error is 0.01547 and the 90th percentile is 0.04129. Centroid checks do
not validate every location on a triangle.

Final multiview processing took 15.69 seconds, versus approximately 7.35 seconds
for the weaker old method. Optimizing the stricter implementation reduced its
own runtime from approximately 34.58 seconds. These are individual local runs,
not a controlled throughput benchmark; improved checks cost additional work.

## Existing physical recording

Reprocessed `runs/rc-20261012-211702-bQiV5D` without replacing old `dense/`
outputs. New results are in `dense_v2/`:

| Independent map | Points | Triangles |
| --- | ---: | ---: |
| map_1 | 21,537 | 5,703 |
| map_2 | 20,210 | 8,794 |
| Total, not a merged coordinate frame | 41,747 | 14,497 |

Processing took 14.60 seconds; previous processing took approximately 8.78
seconds and produced 15,900 points. More points alone do not establish greater
accuracy. No ground truth exists for this room. The images include blur and
moving subjects; the original recording tracked about 65.6% of processed
frames. The reconstruction remains partial, with holes and separate maps.
No new physical tracking run was performed after the tracker changes.

The viewer uses WebGL depth testing, point/surface modes, calibrated camera
alignment, orbit controls and independent-map selection. Saved maps no longer
poll repeatedly. Reference images now correspond to a reconstruction view,
instead of an unrelated final frame. PLY downloads include mesh faces when
available. Screenshots and recordings remain local, outside Git.

## Checks

- 28 Python unit/integration tests passed (4.87 seconds).
- Native CPU runner rebuilt successfully; upstream Eigen warnings remain.
- Updated native runner synthetic smoke test: 220 processed frames, 215 tracked,
  1,894 sparse points and 27 keyframes. Similarity-aligned position RMSE was
  0.00226 scene units. Image-quality telemetry was finite throughout.
- Playwright/Chromium tested rendered WebGL pixels, zero GL errors, surface
  toggle, orbit, camera view, map switching, downloads and no saved-map polling.
- Both map API, image and download endpoints returned 200; unknown maps 404.
- JavaScript and shell syntax, Python compilation and diff whitespace checked.

To reproduce core tests:

```bash
bash run-python.sh -m unittest discover -s tests -p 'test_*.py' -v
bash run-python.sh rc/dense_reconstruct.py runs/YOUR_RECORDED_RUN
bash run-python.sh rc/view_map.py runs/YOUR_RECORDED_RUN
```

The optional browser test is `tests/viewer_browser.mjs`; it requires Playwright
and Chromium. The evaluation helper is `tests/evaluate_rc_scene.py`.

## Next capture requirements

Use a well-lit, static scene; keep the camera rigid, move slowly with sideways
translation and overlapping views, and revisit already observed areas. Avoid
pure rotation and moving people. Inspect sharpness before recording. A single
monocular camera still has unknown physical scale; this implementation must
not be used as a verified metric obstacle map or autonomous flight input.
