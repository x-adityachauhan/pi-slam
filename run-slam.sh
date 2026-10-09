#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CALIBRATION="$PROJECT_DIR/calibration/camera.yaml"
if [[ ! -f "$CALIBRATION" ]]; then
    printf 'Camera calibration is required. Run: bash run-python.sh calibrate.py\n' >&2
    exit 1
fi
mkdir -p "$PROJECT_DIR/runs"
RUN_DIR="$(mktemp -d "$PROJECT_DIR/runs/orb-$(date +%Y%m%d-%H%M%S)-XXXXXX")"
cp -- "$CALIBRATION" "$RUN_DIR/camera.yaml"
printf '\nSystem.SaveAtlasToFile: "atlas"\n' >> "$RUN_DIR/camera.yaml"
cd "$RUN_DIR"
export LD_LIBRARY_PATH="$PROJECT_DIR/env/lib:$PROJECT_DIR/vendor/ORB_SLAM3/lib:$PROJECT_DIR/vendor/ORB_SLAM3/Thirdparty/DBoW2/lib:$PROJECT_DIR/vendor/ORB_SLAM3/Thirdparty/g2o/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export QT_QPA_PLATFORM=xcb
export PANGOLIN_WINDOW_URI='x11://'
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=2
COMMAND=("$PROJECT_DIR/build/pi_slam_live" "$PROJECT_DIR/vendor/ORB_SLAM3/Vocabulary/ORBvoc.txt"
    "$RUN_DIR/camera.yaml" "$RUN_DIR" "$@")
if [[ ${SLAM_DEBUG:-0} == 1 ]]; then
    exec "$PROJECT_DIR/env/bin/gdb" --batch -ex run -ex 'thread apply all bt' --args "${COMMAND[@]}"
fi
exec "${COMMAND[@]}"
