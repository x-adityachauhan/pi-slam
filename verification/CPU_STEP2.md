# Step 2 — tracking candidate

Identical recorded inputs: `rc-20261012-235634-i1l4YD`, 655 frames. Single
headless replay per configuration; asynchronous optimization is nondeterministic.

| Metric | Baseline replay | Candidate replay |
| --- | ---: | ---: |
| Delivered FPS, result span | 12.25 | 12.25 |
| Tracked % including initialization | 58.93 | 90.38 |
| Map count | 1 | 1 |
| Mean keypoints | 1265 | 1673 |
| Mean tracking ms | 20.9 | 25.51 |

Candidate: 2000 features, FAST thresholds 12/5, scale factor 1.2, eight levels.
Visual recovery window 15 seconds, including small initialized monocular maps.
Geometry acceptance thresholds unchanged. Default upstream behavior remains
three seconds unless CPU.VisualRecoverySeconds is specified. This does not
guarantee recovery when the scene no longer overlaps.

The EOF of a replay deliberately reports stream_failed=true and exit code 3.
This is not a physical network failure. Recorded gaps (414 source sequences)
are identical, not fixed by replay. Physical before/after remains pending.

CPU-only camera launcher requests 10 ms exposure, automatic ISO, JPEG quality
75 and restores exposure controls on shutdown. Initial transport probe: mean
gray 83.74/255, JPEG 58,130 bytes, 60 frames spanning 2.95 capture seconds;
7.97 wall seconds including startup. Baseline JPEG mean was 62,897 bytes but
scene changes prevent attributing that difference solely to compression.
Do not claim sustained 20 FPS or verified sensor exposure from this probe.

Run CPU camera with `bash rc/start-camera.sh --deploy`, then `bash rc/run.sh`.
The original camera launcher and GPU entrypoints are unchanged. Camera settings
are an opt-in CPU profile until a repeatable physical trial validates them.

Native rebuild passed; Python suite passed 30 tests with one optional native
test skipped. Reported replay numbers are not physical accuracy measurements.
