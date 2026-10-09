#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
command -v docker >/dev/null || { printf 'Install Docker and NVIDIA Container Toolkit on the RTX laptop first. See README.md.\n' >&2; exit 1; }
docker build -f "$PROJECT_DIR/gpu/Dockerfile" -t pi-slam-rtx5060 "$PROJECT_DIR"
