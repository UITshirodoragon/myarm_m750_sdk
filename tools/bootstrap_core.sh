#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CORE_VENV_DIR="${ROOT_DIR}/.venv-core"
CORE_PYTHON_BIN="${MYARM_CORE_PYTHON:-python3}"
CONSTRAINTS_FILE="${ROOT_DIR}/requirements/constraints-py38.txt"

# A sourced ROS setup commonly injects both ROS and the user site through
# PYTHONPATH. The core environment must not inherit either path.
unset PYTHONHOME PYTHONPATH
export PIP_DISABLE_PIP_VERSION_CHECK=1
export PYTHONNOUSERSITE=1

"${CORE_PYTHON_BIN}" -c '
import sys
if sys.version_info[:2] != (3, 8):
    raise SystemExit(
        "MyArm M750 core gate requires Python 3.8; got {0}.{1}".format(
            sys.version_info[0], sys.version_info[1]
        )
    )
'

if [[ -x "${CORE_VENV_DIR}/bin/python" ]] && ! \
  "${CORE_VENV_DIR}/bin/python" -c '
from pathlib import Path
import pip
import sys

Path(pip.__file__).resolve().relative_to(Path(sys.prefix).resolve())
  '; then
  echo "Recreating contaminated .venv-core (pip resolved outside the venv)."
  "${CORE_PYTHON_BIN}" -m venv --clear "${CORE_VENV_DIR}"
elif [[ ! -x "${CORE_VENV_DIR}/bin/python" ]]; then
  "${CORE_PYTHON_BIN}" -m venv "${CORE_VENV_DIR}"
fi

"${CORE_VENV_DIR}/bin/python" -m pip install \
  --constraint "${CONSTRAINTS_FILE}" \
  pip \
  setuptools \
  wheel

"${CORE_VENV_DIR}/bin/python" -m pip install \
  --constraint "${CONSTRAINTS_FILE}" \
  --no-build-isolation \
  --editable "${ROOT_DIR}/pycore[dev]"

"${CORE_VENV_DIR}/bin/python" -c '
from pathlib import Path
import pip
import sys

if sys.version_info[:2] != (3, 8):
    raise SystemExit(".venv-core was not created with Python 3.8")
Path(pip.__file__).resolve().relative_to(Path(sys.prefix).resolve())
'

echo "Core development environment ready: ${CORE_VENV_DIR}"
