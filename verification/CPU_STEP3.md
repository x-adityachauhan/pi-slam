# Step 3 — CPU live display

Native CPU snapshots publish at most four times/second, capped at 20,000 map
points. Map-update locking is try-lock: skip publication rather than wait for
mapping. JPEG/JSON are atomically replaced. Serialization and file writes still
have bounded overhead in the tracking loop; depth inference is not in this loop.

The dedicated `rc/live_view.py` server and `rc/web/` assets do not change GPU
viewer files. They show camera feature overlays, FPS, tracked percentage,
keypoint count, independent map count, trajectory, orbitable 3D and an X-Z
projection explicitly labelled not gravity-aligned. No unknown region is
declared free. Missing/stale snapshots are distinguished from current data.

Feature overlay uses undistorted keypoints on an undistorted image. Green means
a nonbad observed map association while tracking; orange means other extracted
features. These are not semantic object detections. Web publication is enabled
by the CPU launcher (`--live-web`). Start its printed `rc/live_view.py` command
to attach the browser to that exact run on port 8093.

Same 655-frame replay, display off versus live publication on:

| Metric | Step 2 | Step 3 |
| --- | ---: | ---: |
| Tracked % | 90.38 | 90.53 |
| Mean keypoints | 1673 | 1673 |
| Maps | 1 | 1 |
| Delivery FPS | 12.25 | 12.25 |
| Mean tracking-call time | 25.51 ms | 26.98 ms |

Single runs, with asynchronous optimization and browser activity; not a strict
isolated overhead benchmark. The browser was exercised while replay ran:
status TRACKING, 332 frames, populated map, live feature image, top-down canvas
width 1230 and WebGL error 0. Screenshot inspected locally. Snapshot API tests
cover waiting, current and stale states. 31 Python tests passed, one optional
native regression skipped. Native build and JS/shell syntax checks passed.

Physical repeat trial and dense learned-depth/TSDF step remain pending.
