#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
printf 'Drone bench research only: nonmetric camera-only pose; no flight-controller output.\n' >&2
exec bash "$PROJECT_DIR/run-slam.sh" --pose-events "$@"
