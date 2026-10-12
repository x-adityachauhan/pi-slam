# CPU live diagnosis — 2026-10-12

Read-only inspection and unchanged physical baseline preceded tuning. Existing
work was preserved as commit 642472d. GPU/DPV code is out of scope.

## Measured baseline

Fresh 60-second run: `runs/rc-20261012-235634-i1l4YD`.

| Metric | Baseline |
| --- | ---: |
| Processed / tracked | 655 / 398 |
| Tracked, including initialization | 60.76% |
| Delivery FPS (result timestamp span) | 10.92 |
| Mean tracking call | 22.39 ms |
| Keypoints/frame mean / median | 1265.07 / 1195 |
| Keypoints p10 / p90 | 802.8 / 1917.6 |
| Independent retained maps | 1 |
| Initialization time (capture clock) | 12.90 s |
| Skipped source sequence numbers | 414 |
| Capture gap p95 / max | 199.94 / 3499.02 ms |
| Receipt-to-result median / p95 | 25.43 / 44.78 ms |
| Sparse points / keyframes | 631 / 18 |

The older user-quoted run (2026-10-09) processed 1199 frames, tracked 414
(34.53%), skipped one and retained three maps. It is not the latest baseline.
The preceding October 12 run processed 1305, tracked 555 (42.53%), skipped 310
and retained one map. Different scenes/motion mean these are not controlled
before/after measurements.

## Causes confirmed in code or measurements

1. Camera display: `cpp/live.cc` displays a raw frame clone with text, but draws
   no feature circles. Invisible feature detections are a display omission,
   not proof the extractor has no detections.
2. Web transport: native CPU map export runs at shutdown. Pose telemetry does
   not publish live map geometry. Both running web servers are saved-run viewers.
   `/api` on 8090 returned 21,537 old points with `saved=true`; 8092 returned
   all 882 latest sparse points with `saved=true`. The page intentionally stops
   polling saved maps. No live CPU-to-web map publisher exists. Data is not
   being silently dropped in that missing bridge.
3. Delivery gaps: baseline capture timestamps are monotonic, but source
   sequence gaps total 414, with gaps up to 3.50 seconds. Tracking costs ~22 ms,
   below the 50 ms camera period. Post-run SSH TCP statistics showed 88 ms RTT,
   75 ms RTT variation, nine local retransmissions and 5,133 received out-of-order
   packets over the long-lived connection. Those aggregate TCP counters do not
   isolate this run, but support investigating Wi-Fi/transport. Receiver latest-
   frame selection also intentionally skips stale frames. CPU tracking alone
   does not explain the delivery rate. No synchronized clocks: receipt-to-result
   is not end-to-end latency and clock-offset subtraction would be misleading.
4. Recovery: `Tracking.cc` calls Relocalization in RECENTLY_LOST, but visual
   tracking moves to LOST after three capture seconds without recovery. Failed
   prediction on maps with <=10 keyframes can go directly to LOST, and the end
   of Track resets such a map. Larger lost maps trigger CreateMapInAtlas. The
   old and fresh logs contain successful `Relocalized!!` events: recovery is
   already attempted, but small-map policy and the short window are limiting.
5. Dense output: fresh baseline accepted only three stereo pairs and no depth
   survived multi-view consistency. The previous run had 8,021 candidate depth
   pixels but zero validated pixels. Sparse tracking success does not establish
   reliable dense geometry. Relaxing validation to draw surfaces is not a fix.

## Parameters and camera

ORB settings: nFeatures=1600, scaleFactor=1.2, nLevels=8, iniThFAST=20,
minThFAST=7. The monocular initialization extractor can exceed the normal
feature budget; a count above 1600 is not inherently erroneous. No evidence yet
that changing any one setting improves this camera's tracking.

Pi `/dev/video0`: MMAL driver, 640x480 MJPEG, 20 FPS, quality 90,
brightness=50/100, auto_exposure=0 (Auto), ISO auto enabled, exposure bias zero,
50 Hz anti-flicker, dynamic frame rate disabled. The exposure control reads
1000 in auto mode; this is not reliable measured per-frame exposure. Actual
sensor gain is not exposed by these controls. Do not claim it was measured.

Original `verification/live_camera.jpg`: mean grayscale 22.88/255, 86.43% of
pixels below 30. A recent room frame averaged 88.52/255, with 17.89% below 30;
the recent run's median reported brightness was 99.63/255. Original darkness
is proven; current total failure cannot be attributed solely to darkness.
Visible softness remains, but motion blur versus optical defocus is not yet
isolated. Laplacian variance is texture/noise-dependent, not a focus guarantee.

## Ordered next work

2. Instrument delivered FPS, gaps and recovery; evaluate CPU parameter and
   bounded-exposure candidates on a saved replay and repeatable physical route.
   Extend visual relocalization opportunity without accepting invalid poses.
3. Add tracked/untracked pixel overlays, bounded live map snapshots and a CPU-
   specific web bridge, live 3D/trajectory and explicitly oriented top-down view.
   Keep the GPU entrypoints and their viewer behavior unchanged.
4. Add CPU-only small-model depth asynchronously on keyframes, robust inverse-
   depth alignment to SLAM observations, gated coarse TSDF fusion and a range
   projection. Learned depths are predictions, not measured LiDAR; scale and
   gravity are not observable from this camera alone. Map resets and loop
   corrections must invalidate/reintegrate affected dense geometry.

Report delivery FPS, tracked fraction, map count and keypoints for every stage.
Use replay to compare identical inputs and physical trials to assess camera
changes. Do not invent missing before/after measurements.
