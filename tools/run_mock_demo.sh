#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python3 -m pip install --no-deps --no-build-isolation -e "${ROOT_DIR}/pycore" >/dev/null
cd "${ROOT_DIR}"
python3 examples/mock_joint_demo.py --config pycore/config/default.yaml
