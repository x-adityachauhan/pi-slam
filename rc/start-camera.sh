#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
CPU_PI_TARGET=ras@10.32.62.210
CPU_PI_ALIAS=
if [[ -f "$PROJECT_DIR/.pi-target" ]]; then
    read -r CPU_PI_TARGET CPU_PI_ALIAS < "$PROJECT_DIR/.pi-target"
fi
if [[ -n ${PI_TARGET:-} ]]; then CPU_PI_ALIAS=; fi
CPU_PI_TARGET="${PI_TARGET:-$CPU_PI_TARGET}"
CPU_PI_ALIAS="${PI_HOST_KEY_ALIAS:-$CPU_PI_ALIAS}"
CPU_SSH=(-F /dev/null -o ConnectTimeout=8 -o ConnectionAttempts=1)
[[ -z "$CPU_PI_ALIAS" ]] || CPU_SSH+=(-o "HostKeyAlias=$CPU_PI_ALIAS")
if [[ ${1:-} == --deploy ]]; then
    scp "${CPU_SSH[@]}" "$PROJECT_DIR/pi/camera_server.py" "$PROJECT_DIR/pi/cpu_camera_server.py" "$CPU_PI_TARGET:/home/ras/pi-slam/"
fi
exec ssh -tt "${CPU_SSH[@]}" -o ExitOnForwardFailure=yes -o ServerAliveInterval=5 -o ServerAliveCountMax=3 \
    -L 127.0.0.1:8765:127.0.0.1:8765 "$CPU_PI_TARGET" \
    'exec python3 -u /home/ras/pi-slam/cpu_camera_server.py'
