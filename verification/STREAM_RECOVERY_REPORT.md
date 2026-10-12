# Stream-failure handling update — 2026-10-12

The physical run `rc-20261012-232356-k6NU1S` recorded 435 pose events, all
uninitialized, spanning 24.99 capture seconds. It then reported a disconnected
or timed-out stream. Its exception path skipped the normal summary and exited
before the launcher's reconstruction step. The available log does not identify
whether Wi-Fi, SSH or the Pi caused the interruption.

OpenCV diagnostic feature detection on 22 sampled images found a median of
1,574.5 features with a 1,600-feature budget and FAST threshold 7. This is not
the native ORB-SLAM3 extractor or proof of usable geometric matches. The exact
initialization failure remains unproven; no geometry acceptance thresholds were
lowered, and no successful new physical map is claimed.

Changes:

- Preserve partial socket reads across transient receive timeouts, with a
  15-second no-progress limit and one-second interrupt checks.
- Distinguish EOF, stalled reception and socket errors in diagnostics.
- Increase Pi blocked-send tolerance from two to ten seconds; log disconnects.
- On stream failure, save summary, error text and available maps. The launcher
  reconstructs usable partial maps and explicitly reports uninitialized runs.
- Expose native feature counts and flush initialization/progress messages.

Verification:

- Native build succeeded (existing upstream Eigen deprecation warnings).
- Existing 28 Python tests passed.
- New opt-in native regression passed: a header split across a 1.3-second pause
  remained intact, all three frames were processed, EOF returned status 3, and
  summary/error/feature telemetry were saved despite no initialized map.
- Synthetic 100-frame source deliberately closed the connection. The runner
  initialized, saved 1,408 reliable points and 19 keyframes, and returned status
  3 with the explicit connection-closed diagnostic.
- Deployed the camera-server change to the Pi. A live transport check received
  100 frames with no sequence gaps and capture timestamps indicating 20.01 FPS.
  The check took 11.40 wall-clock seconds including connection/startup, so this
  does not establish low network latency or sustained real-time delivery.

Run the native regression with:

```bash
PI_SLAM_NATIVE_TEST=1 bash run-python.sh -m unittest discover -s tests -p test_native_stream.py -v
```

This is graceful partial-result recovery, not automatic reconnection. A new
physical mapping trial is still required to assess initialization and room
coverage after deployment.
