#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PI_TARGET="${PI_TARGET:-ras@10.32.62.210}"
if [[ ${1:-} == --deploy ]]; then
    ssh -F /dev/null "$PI_TARGET" 'mkdir -p /home/ras/pi-slam'
    scp -F /dev/null "$PROJECT_DIR/pi/camera_server.py" "$PI_TARGET:/home/ras/pi-slam/camera_server.py"
fi
exec ssh -tt -F /dev/null -o ExitOnForwardFailure=yes -o ServerAliveInterval=15 \
    -L 127.0.0.1:8765:127.0.0.1:8765 "$PI_TARGET" \
    'exec python3 -u /home/ras/pi-slam/camera_server.py --width 640 --height 480 --fps 20 --quality 90'
