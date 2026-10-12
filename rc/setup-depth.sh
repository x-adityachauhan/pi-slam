#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_DIR"
bash run-python.sh -m pip install openvino==2026.4.1 scikit-image==0.26.0
mkdir -p models/cpu
curl -fL --retry 2 https://github.com/isl-org/MiDaS/releases/download/v3_1/openvino_midas_v21_small_256.xml -o models/cpu/midas-small.xml
curl -fL --retry 2 https://github.com/isl-org/MiDaS/releases/download/v3_1/openvino_midas_v21_small_256.bin -o models/cpu/midas-small.bin
sha256sum -c rc/depth-model.sha256
