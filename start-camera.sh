#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_PI_TARGET=ras@10.32.62.210
CONFIG_PI_HOST_KEY_ALIAS=
if [[ -f "$PROJECT_DIR/.pi-target" ]]; then
    read -r CONFIG_PI_TARGET CONFIG_PI_HOST_KEY_ALIAS < "$PROJECT_DIR/.pi-target"
fi
if [[ -n ${PI_TARGET:-} ]]; then
    CONFIG_PI_HOST_KEY_ALIAS=
fi
PI_TARGET="${PI_TARGET:-$CONFIG_PI_TARGET}"
SSH_OPTIONS=(-F /dev/null -o ConnectTimeout=8 -o ConnectionAttempts=1)
PI_HOST_KEY_ALIAS="${PI_HOST_KEY_ALIAS:-$CONFIG_PI_HOST_KEY_ALIAS}"
[[ -z "$PI_HOST_KEY_ALIAS" ]] || SSH_OPTIONS+=(-o "HostKeyAlias=$PI_HOST_KEY_ALIAS")
if [[ ${1:-} == --deploy ]]; then
    ssh "${SSH_OPTIONS[@]}" "$PI_TARGET" 'mkdir -p /home/ras/pi-slam'
    scp "${SSH_OPTIONS[@]}" "$PROJECT_DIR/pi/camera_server.py" "$PI_TARGET:/home/ras/pi-slam/camera_server.py"
fi
exec ssh -tt "${SSH_OPTIONS[@]}" -o ExitOnForwardFailure=yes -o ServerAliveInterval=15 \
    -L 127.0.0.1:8765:127.0.0.1:8765 "$PI_TARGET" \
    'exec python3 -u /home/ras/pi-slam/camera_server.py --width 640 --height 480 --fps 20 --quality 90'
