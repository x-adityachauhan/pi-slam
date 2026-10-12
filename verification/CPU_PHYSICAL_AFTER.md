# CPU physical after-test — 2026-10-13

Run: `runs/rc-20261013-002236-x7vEy1`, 60-second requested capture, CPU-only
tracking with native viewer, live web publication and separate depth worker.
Camera profile: requested 10 ms exposure, automatic ISO, 640x480 at 20 FPS,
JPEG quality 75. User instructed to repeat the sideways-and-back baseline route.
Motion and lighting were not instrumented identically; this is not a controlled
accuracy benchmark or attribution of gains to any single change.

| Metric | Physical baseline | Physical after |
| --- | ---: | ---: |
| Processed / tracked | 655 / 398 | 1013 / 1000 |
| Tracked % including startup | 60.76 | 98.72 |
| Delivered FPS, result span | 10.92 | 16.89 |
| Mean keypoints/frame | 1265 | 1883 |
| Map count | 1 | 1 |
| Initialization, capture seconds | 12.90 | 1.00 |
| Maximum capture gap | 3499 ms | 150 ms |
| Skipped source sequences | 414 | 187 |
| Mean tracking call | 22.39 ms | 42.34 ms |
| Receipt-to-result p95 | 44.78 ms | 101.27 ms |

After initialization every processed frame was tracked: no lost-state events,
no map resets and no stream failure. Greater CPU load and receipt-to-result
latency remain a tradeoff; these are not end-to-end latency measurements.

Outputs:

- 1187 reliable sparse landmarks, 12 optimized keyframes.
- Final learned-depth local TSDF snapshot: 12 accepted keyframes, 3890
  triangles, 1.27-second worker update. Held-out alignment median relative
  errors ranged about 2.6–7.7%. This is agreement with SLAM anchors, not physical
  surface error. Coarse shape remains visibly incomplete/oversmoothed.
- Final multi-view post-processing: 23556 colored points, 10925 triangles,
  29 successful stereo pairs of 31 attempted, 8.03 seconds. This is separate
  from learned TSDF, not a count to combine with it.
- Browser checked during real capture: active changing state, feature overlays,
  populated predicted mesh, X-Z panel and projected range bins; zero WebGL
  error, no displayed error. Screenshot inspected and retained locally.

Live URL 8093 is attached to this run and becomes explicitly stale after stop.
Saved multi-view reconstruction is on 8094. Camera orientation was not confirmed:
top-down remains a map-axis projection, not a gravity-aligned navigation map.
Monocular scale is arbitrary; no metric accuracy, full-room coverage or safe
free-space claim is made. Raw images and screenshots are not committed.
