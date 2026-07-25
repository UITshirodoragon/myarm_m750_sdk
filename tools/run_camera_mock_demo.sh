#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

CORE_PYTHON="${ROOT_DIR}/.venv-core/bin/python"
if [[ ! -x "${CORE_PYTHON}" ]]; then
  echo "Missing .venv-core. Run ./tools/bootstrap_core.sh first." >&2
  exit 2
fi

unset PYTHONHOME PYTHONPATH
export PYTHONNOUSERSITE=1

cd "${ROOT_DIR}"
"${CORE_PYTHON}" examples/camera_standalone_demo.py
