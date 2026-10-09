#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$PROJECT_DIR/runs"
if [[ ${1:-} == --doctor ]]; then
    exec docker run --rm --gpus all --network host --shm-size=1g \
        pi-slam-rtx5060 python gpu/doctor.py
fi
[[ -f "$PROJECT_DIR/calibration/intrinsics.json" ]] || { printf 'Copy the actual camera calibration/intrinsics.json from the first laptop.\n' >&2; exit 1; }
exec docker run --rm -it --gpus all --network host --shm-size=1g \
    --user "$(id -u):$(id -g)" -v "$PROJECT_DIR/calibration:/data/calibration:ro" \
    -v "$PROJECT_DIR/runs:/data/runs" -e HOME=/tmp -e NUMBA_CACHE_DIR=/tmp/numba \
    pi-slam-rtx5060 python gpu/live_dpvo.py --calibration /data/calibration/intrinsics.json \
    --output /data/runs "$@"
