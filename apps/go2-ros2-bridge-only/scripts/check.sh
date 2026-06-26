#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

wendy json validate "$ROOT"
bash -n "$ROOT/bridge/entrypoint.sh"

python3 -m py_compile \
  "$ROOT"/bridge/ros2_ws/src/go2_bridge/go2_bridge/*.py \
  "$ROOT"/bridge/ros2_ws/src/go2_bridge/launch/*.py

find "$ROOT" -type d -name __pycache__ -prune -exec rm -rf {} +

echo "bridge-only checks passed"
