#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python3 -m pip install --no-deps --no-build-isolation -e "${ROOT_DIR}/pycore" >/dev/null
python3 -m pytest -q "${ROOT_DIR}/pycore/tests"
