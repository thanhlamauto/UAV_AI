#!/usr/bin/env bash
set -euo pipefail

PORT="${1:-8765}"
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

cd "$ROOT_DIR"

PYTHON_BIN=""
for candidate in "$ROOT_DIR/.venv/bin/python" "/tmp/uav_oda_ros2_check/bin/python" "python3"; do
  if command -v "$candidate" >/dev/null 2>&1; then
    if "$candidate" - <<'PY' >/dev/null 2>&1
import numpy
PY
    then
      PYTHON_BIN="$candidate"
      break
    fi
  fi
done

if [[ -z "$PYTHON_BIN" ]]; then
  echo "Could not find a Python interpreter with numpy installed." >&2
  echo "Create/activate a venv and install requirements.txt, then rerun this script." >&2
  exit 1
fi

"$PYTHON_BIN" scripts/export_3d_simulation_data.py
echo "Serving 3D simulation at http://localhost:${PORT}/"
"$PYTHON_BIN" -m http.server "$PORT" --directory sim3d
