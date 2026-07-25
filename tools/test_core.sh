#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CORE_PYTHON="${ROOT_DIR}/.venv-core/bin/python"

if [[ ! -x "${CORE_PYTHON}" ]]; then
  echo "Missing .venv-core. Run ./tools/bootstrap_core.sh first." >&2
  exit 2
fi

unset PYTHONHOME PYTHONPATH
export PIP_DISABLE_PIP_VERSION_CHECK=1
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export PYTHONNOUSERSITE=1

QUALITY_TEMP_DIR="$(mktemp -d "${TMPDIR:-/tmp}/myarm-m750-core-gate.XXXXXXXX")"
trap 'rm -rf "${QUALITY_TEMP_DIR}"' EXIT

"${CORE_PYTHON}" -m ruff check \
  --config "${ROOT_DIR}/pycore/pyproject.toml" \
  "${ROOT_DIR}/pycore/src" \
  "${ROOT_DIR}/pycore/tests" \
  "${ROOT_DIR}/tools/check_coverage.py" \
  "${ROOT_DIR}/tools/model" \
  "${ROOT_DIR}/tools/moveit_runtime_gate.py" \
  "${ROOT_DIR}/tools/ros_runtime_gate.py" \
  "${ROOT_DIR}/ros2/src/myarm_m750_moveit_config/test/moveit_runtime_probe.py" \
  "${ROOT_DIR}/tools/verify_release.py" \
  "${ROOT_DIR}/tools/wheel_smoke.py"

"${CORE_PYTHON}" -m pytest \
  -q \
  -o addopts='' \
  -p pytest_cov \
  --cov=myarm_m750_core \
  --cov-report=term-missing \
  --cov-report="json:${QUALITY_TEMP_DIR}/coverage.json" \
  --cov-fail-under=85 \
  "${ROOT_DIR}/pycore/tests"

"${CORE_PYTHON}" "${ROOT_DIR}/tools/check_coverage.py" \
  "${QUALITY_TEMP_DIR}/coverage.json" \
  --minimum 90

"${CORE_PYTHON}" "${ROOT_DIR}/tools/verify_release.py"

PYTHONPYCACHEPREFIX="${QUALITY_TEMP_DIR}/pycache" \
  "${CORE_PYTHON}" -m compileall -q \
    "${ROOT_DIR}/pycore/src" \
    "${ROOT_DIR}/examples" \
    "${ROOT_DIR}/benchmarks"

mkdir -p "${QUALITY_TEMP_DIR}/wheel" "${QUALITY_TEMP_DIR}/site"
"${CORE_PYTHON}" -m pip wheel \
  --disable-pip-version-check \
  --no-build-isolation \
  --no-deps \
  --wheel-dir "${QUALITY_TEMP_DIR}/wheel" \
  "${ROOT_DIR}/pycore"

mapfile -t CORE_WHEELS < <(
  find "${QUALITY_TEMP_DIR}/wheel" -maxdepth 1 -type f -name 'myarm_m750_core-*.whl'
)
if [[ "${#CORE_WHEELS[@]}" -ne 1 ]]; then
  echo "Expected exactly one core wheel, found ${#CORE_WHEELS[@]}." >&2
  exit 1
fi

"${CORE_PYTHON}" -m pip install \
  --disable-pip-version-check \
  --no-deps \
  --target "${QUALITY_TEMP_DIR}/site" \
  "${CORE_WHEELS[0]}"

(
  cd "${QUALITY_TEMP_DIR}/site"
  "${CORE_PYTHON}" -m mypy \
    --config-file "${ROOT_DIR}/pycore/pyproject.toml" \
    --package myarm_m750_core
)

MYARM_WHEEL_SITE="${QUALITY_TEMP_DIR}/site" \
PYTHONPATH="${QUALITY_TEMP_DIR}/site" \
  "${CORE_PYTHON}" -c '
import os
from pathlib import Path

import myarm_m750_core

module_path = Path(myarm_m750_core.__file__).resolve()
wheel_site = Path(os.environ["MYARM_WHEEL_SITE"]).resolve()
module_path.relative_to(wheel_site)
print("PASS wheel import:", module_path)
'

MYARM_WHEEL_SITE="${QUALITY_TEMP_DIR}/site" \
PYTHONPATH="${QUALITY_TEMP_DIR}/site" \
  "${CORE_PYTHON}" "${ROOT_DIR}/tools/wheel_smoke.py"

echo "Core quality gate passed."
