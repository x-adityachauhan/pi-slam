#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "$(uname -s)" != Linux || "$(uname -m)" != x86_64 ]]; then
  echo "The CPU build requires Linux x86_64." >&2
  exit 1
fi
for required in git curl tar c++; do
  command -v "$required" >/dev/null || { echo "Missing prerequisite: $required" >&2; exit 1; }
done
mkdir -p "$PROJECT_DIR/tools" "$PROJECT_DIR/vendor"
MAMBA="$PROJECT_DIR/tools/bin/micromamba"
if [[ ! -x "$MAMBA" ]]; then
  curl -fL --retry 2 https://micro.mamba.pm/api/micromamba/linux-64/latest -o "$PROJECT_DIR/tools/micromamba.tar.bz2"
  tar -xjf "$PROJECT_DIR/tools/micromamba.tar.bz2" -C "$PROJECT_DIR/tools" bin/micromamba
fi
export MAMBA_ROOT_PREFIX="$PROJECT_DIR/mamba"
if [[ ! -f "$PROJECT_DIR/env/conda-meta/history" ]]; then
  "$MAMBA" create -y -p "$PROJECT_DIR/env" -f "$PROJECT_DIR/environment-explicit.txt"
fi

prepare_source() {
  local name="$1" remote="$2" revision="$3" patch="$4"
  local source_dir="$PROJECT_DIR/vendor/$name"
  if [[ ! -d "$source_dir" ]]; then
    mkdir -p "$source_dir"
    git -C "$source_dir" init -q
    git -C "$source_dir" remote add origin "$remote"
    git -C "$source_dir" fetch --depth 1 origin "$revision"
    git -C "$source_dir" checkout --detach FETCH_HEAD
  fi
  if [[ "$(git -C "$source_dir" rev-parse HEAD)" != "$revision" ]]; then
    echo "Unexpected revision in $source_dir; preserving the existing checkout." >&2
    exit 1
  fi
  if git -C "$source_dir" apply --reverse --check "$patch" >/dev/null 2>&1; then
    echo "$name: build patch already applied."
  else
    git -C "$source_dir" apply --check "$patch"
    git -C "$source_dir" apply "$patch"
  fi
}
prepare_source ORB_SLAM3 https://github.com/UZ-SLAMLab/ORB_SLAM3.git 4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4 "$PROJECT_DIR/verification/orb-kali.patch"
prepare_source Pangolin https://github.com/stevenlovegrove/Pangolin.git aff6883c83f3fd7e8268a9715e84266c42e2efe3 "$PROJECT_DIR/verification/pangolin-kali.patch"
RECOVERY_PATCH="$PROJECT_DIR/verification/orb-cpu-recovery.patch"
if ! git -C "$PROJECT_DIR/vendor/ORB_SLAM3" apply --reverse --check "$RECOVERY_PATCH" >/dev/null 2>&1; then
  git -C "$PROJECT_DIR/vendor/ORB_SLAM3" apply --check "$RECOVERY_PATCH"
  git -C "$PROJECT_DIR/vendor/ORB_SLAM3" apply "$RECOVERY_PATCH"
fi
exec bash "$PROJECT_DIR/build-cpu.sh"
