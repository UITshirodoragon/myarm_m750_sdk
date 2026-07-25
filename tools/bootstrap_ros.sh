#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_VENV_DIR="${ROOT_DIR}/.venv-ros"
ROS_SETUP_FILE="/opt/ros/foxy/setup.bash"
ROS_PYTHON_BIN="${MYARM_ROS_PYTHON:-/usr/bin/python3}"
CONSTRAINTS_FILE="${ROOT_DIR}/requirements/constraints-py38.txt"

unset PYTHONHOME PYTHONPATH
export PIP_DISABLE_PIP_VERSION_CHECK=1
export PYTHONNOUSERSITE=1

if [[ ! -f "${ROS_SETUP_FILE}" ]]; then
  echo "ROS 2 Foxy is required at ${ROS_SETUP_FILE}." >&2
  exit 2
fi

"${ROS_PYTHON_BIN}" -c '
import sys
if sys.version_info[:2] != (3, 8):
    raise SystemExit(
        "ROS Foxy gate requires Python 3.8; got {0}.{1}".format(
            sys.version_info[0], sys.version_info[1]
        )
    )
'

if [[ ! -x "${ROS_VENV_DIR}/bin/python" ]]; then
  "${ROS_PYTHON_BIN}" -m venv --system-site-packages "${ROS_VENV_DIR}"
fi

"${ROS_VENV_DIR}/bin/python" -m pip install \
  --constraint "${CONSTRAINTS_FILE}" \
  importlib-metadata \
  packaging \
  pip \
  setuptools \
  wheel \
  zipp

# ROS Foxy's launch_testing plugin is incompatible with current pytest 8.
# Keep this pin isolated from the core environment.
"${ROS_VENV_DIR}/bin/python" -m pip install "pytest==6.2.5"
"${ROS_VENV_DIR}/bin/python" -m pip install \
  --no-build-isolation \
  --no-deps \
  --editable "${ROOT_DIR}/pycore"

# shellcheck disable=SC1091
set +u
source "${ROS_SETUP_FILE}"
set -u
"${ROS_VENV_DIR}/bin/python" -c '
from pathlib import Path
import pip
import sys

import rclpy
import myarm_m750_core

Path(pip.__file__).resolve().relative_to(Path(sys.prefix).resolve())
print("ROS Python:", rclpy.__file__)
print("MyArm core:", myarm_m750_core.__file__)
'

echo "ROS Foxy development environment ready: ${ROS_VENV_DIR}"
