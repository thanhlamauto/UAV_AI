#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

PYTHON_BIN="${PYTHON_BIN:-$ROOT_DIR/.venv/bin/python}"

if [[ ! -x "$PYTHON_BIN" ]]; then
  echo "Creating local .venv for Phase 0 ESDF demo"
  python3 -m venv "$ROOT_DIR/.venv"
  PYTHON_BIN="$ROOT_DIR/.venv/bin/python"
fi

"$PYTHON_BIN" - <<'PY' >/dev/null 2>&1 || "$PYTHON_BIN" -m pip install numpy matplotlib
import matplotlib
import numpy
PY

"$PYTHON_BIN" experiments/run_3d_esdf_mppi_demo.py
"$PYTHON_BIN" scripts/check_3d_esdf_mppi_demo.py

echo
echo "Phase 0 3D ESDF MPPI proof complete."
echo "Open: outputs/indoor_3d_esdf_mppi_summary.md"

