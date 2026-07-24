#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python3 -m pip install --no-deps --no-build-isolation -e "${ROOT_DIR}/pycore" >/dev/null
cd "${ROOT_DIR}"
python3 examples/camera_standalone_demo.py
