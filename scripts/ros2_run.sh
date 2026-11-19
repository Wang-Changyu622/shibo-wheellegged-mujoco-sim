#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
python_bin="${WHEELLEGGED_PYTHON:-$project_root/.venv/bin/python}"
export PYTHONPATH="$project_root/src:$project_root/ros2/wheellegged_ros${PYTHONPATH:+:$PYTHONPATH}"
exec "$python_bin" -m wheellegged_ros.sim_node "$@"
