#!/usr/bin/env bash
set -euo pipefail
[[ $# -ge 1 && -d "$1/dense" ]] || { printf 'Usage: bash gpu/view.sh RUN_DIRECTORY [--map map_0]\n' >&2; exit 1; }
SESSION_DIR="$(realpath -- "$1")"
shift
exec docker run --rm -it --network host --user "$(id -u):$(id -g)" \
    -v "$SESSION_DIR:/data/session:ro" -e HOME=/tmp pi-slam-rtx5060 \
    python rc/view_map.py /data/session --no-open "$@"
