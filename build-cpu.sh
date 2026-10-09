#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CMAKE="$PROJECT_DIR/env/bin/cmake"
JOBS="${BUILD_JOBS:-3}"
export LD_LIBRARY_PATH="$PROJECT_DIR/env/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
"$CMAKE" -S "$PROJECT_DIR/vendor/Pangolin" -B "$PROJECT_DIR/vendor/Pangolin/build" \
    -DCMAKE_PREFIX_PATH="$PROJECT_DIR/env" -DCMAKE_INSTALL_PREFIX="$PROJECT_DIR/env" \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS='-include cstdint' \
    -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF -DBUILD_PANGOLIN_PYTHON=OFF \
    -DBUILD_PANGOLIN_FFMPEG=OFF -DBUILD_PANGOLIN_LIBOPENEXR=OFF -DPI_SLAM_WAYLAND=OFF
"$CMAKE" --build "$PROJECT_DIR/vendor/Pangolin/build" -j "$JOBS"
"$CMAKE" --install "$PROJECT_DIR/vendor/Pangolin/build"
for component in DBoW2 g2o; do
    "$CMAKE" -S "$PROJECT_DIR/vendor/ORB_SLAM3/Thirdparty/$component" \
        -B "$PROJECT_DIR/vendor/ORB_SLAM3/Thirdparty/$component/build" \
        -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$PROJECT_DIR/env" -DCMAKE_CXX_FLAGS='-std=c++17 -include cstdint'
    "$CMAKE" --build "$PROJECT_DIR/vendor/ORB_SLAM3/Thirdparty/$component/build" -j "$JOBS"
done
[[ -f "$PROJECT_DIR/vendor/ORB_SLAM3/Vocabulary/ORBvoc.txt" ]] || \
    tar -xzf "$PROJECT_DIR/vendor/ORB_SLAM3/Vocabulary/ORBvoc.txt.tar.gz" -C "$PROJECT_DIR/vendor/ORB_SLAM3/Vocabulary"
"$CMAKE" -S "$PROJECT_DIR/vendor/ORB_SLAM3" -B "$PROJECT_DIR/vendor/ORB_SLAM3/build" \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$PROJECT_DIR/env" -DCMAKE_CXX_FLAGS='-include cstdint -include list'
"$CMAKE" --build "$PROJECT_DIR/vendor/ORB_SLAM3/build" --target ORB_SLAM3 -j "$JOBS"
"$CMAKE" -S "$PROJECT_DIR/cpp" -B "$PROJECT_DIR/build" -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$PROJECT_DIR/env"
"$CMAKE" --build "$PROJECT_DIR/build" -j "$JOBS"
