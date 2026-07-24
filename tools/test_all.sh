#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# The physical src layout is mapped to the myarm_m750_core namespace by
# setuptools. Install the local package before running tests instead of adding
# pycore/src directly to PYTHONPATH.
python3 -m pip install --no-deps --no-build-isolation -e "${ROOT_DIR}/pycore" >/dev/null

python3 -m pytest -q "${ROOT_DIR}/pycore/tests"
python3 "${ROOT_DIR}/tools/verify_release.py"
python3 -m compileall -q \
  "${ROOT_DIR}/pycore/src" \
  "${ROOT_DIR}/examples" \
  "${ROOT_DIR}/benchmarks"

"${ROOT_DIR}/tools/clean.sh"
echo "All v0.1.1 pure-Python tests and release checks passed."
