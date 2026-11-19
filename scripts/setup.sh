#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd -- "$project_root"
python_bin="${WHEELLEGGED_PYTHON:-python3}"
if [[ ! -x .venv/bin/python ]]; then
  "$python_bin" -m venv --system-site-packages .venv
fi
.venv/bin/python -m pip install -e '.[test]'
.venv/bin/python scripts/fetch_assets.py
.venv/bin/python -c 'import mujoco; print("MuJoCo", mujoco.__version__)'
