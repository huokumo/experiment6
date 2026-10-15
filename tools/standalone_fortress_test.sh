#!/usr/bin/env bash
# Build/test the actual simulation plugin without installing ROS 2.
set -euo pipefail
source_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="${1:-${source_root}/build/standalone}"
cmake -S "$source_root" -B "$build_dir" -DSTANDALONE_PLUGIN=ON -DBUILD_TESTING=ON
cmake --build "$build_dir" -j2
IGN_PARTITION="experiment6-test-$$" ctest --test-dir "$build_dir" --output-on-failure
