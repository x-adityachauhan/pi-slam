#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
[[ -f "$PROJECT_DIR/calibration/camera.yaml" ]] || { printf 'Camera calibration missing.\n' >&2; exit 1; }
mkdir -p "$PROJECT_DIR/runs"
RUN_DIR="$(mktemp -d "$PROJECT_DIR/runs/rc-$(date +%Y%m%d-%H%M%S)-XXXXXX")"
cp -- "$PROJECT_DIR/calibration/camera.yaml" "$RUN_DIR/camera.yaml"
bash "$PROJECT_DIR/run-python.sh" rc/cpu_settings.py "$PROJECT_DIR/calibration/camera.yaml" "$RUN_DIR/camera.yaml" --profile candidate
cp -- "$PROJECT_DIR/calibration/intrinsics.json" "$RUN_DIR/intrinsics.json"
printf '\nSystem.SaveAtlasToFile: "atlas"\n' >> "$RUN_DIR/camera.yaml"
cd "$RUN_DIR"
export LD_LIBRARY_PATH="$PROJECT_DIR/env/lib:$PROJECT_DIR/vendor/ORB_SLAM3/lib:$PROJECT_DIR/vendor/ORB_SLAM3/Thirdparty/DBoW2/lib:$PROJECT_DIR/vendor/ORB_SLAM3/Thirdparty/g2o/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export QT_QPA_PLATFORM=xcb PANGOLIN_WINDOW_URI='x11://' OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2
printf 'RC-car research run: %s\nRelative scale; no steering or flight-controller output.\n' "$RUN_DIR"
printf 'Live browser: bash run-python.sh rc/live_view.py %q --port 8093\n' "$RUN_DIR"
trap ':' INT
TRACKER_STATUS=0
"$PROJECT_DIR/build/pi_slam_live" "$PROJECT_DIR/vendor/ORB_SLAM3/Vocabulary/ORBvoc.txt" \
    "$RUN_DIR/camera.yaml" "$RUN_DIR" --record-frames --pose-events --live-web --max-frames 3000 "$@" || TRACKER_STATUS=$?
trap - INT
if (( TRACKER_STATUS != 0 && TRACKER_STATUS != 3 )); then
    printf 'Tracker failed (status %s). Recording retained: %s\n' "$TRACKER_STATUS" "$RUN_DIR" >&2
    exit "$TRACKER_STATUS"
fi
if ! bash "$PROJECT_DIR/run-python.sh" -c 'import json,sys; s=json.load(open(sys.argv[1])); print("Capture summary:", s); sys.exit(0 if s["tracked_frames"] else 1)' "$RUN_DIR/stats.json"; then
    printf 'No initialized map: no 3D result was produced. Recording retained: %s\n' "$RUN_DIR" >&2
    exit 4
fi
if (( TRACKER_STATUS == 3 )); then
    printf 'Stream interrupted; reconstructing the saved partial map. See capture_error.txt.\n' >&2
fi
printf 'SLAM saved. Reconstructing colored 3D surfaces from final optimized poses...\n'
bash "$PROJECT_DIR/run-python.sh" rc/dense_reconstruct.py "$RUN_DIR" --calibration "$RUN_DIR/intrinsics.json"
printf 'Inspect: bash run-python.sh rc/view_map.py %q\n' "$RUN_DIR"
