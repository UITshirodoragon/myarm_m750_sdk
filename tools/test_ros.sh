#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP_FILE="/opt/ros/foxy/setup.bash"
ROS_VENV_DIR="${ROOT_DIR}/.venv-ros"
ROS_COLCON=("${ROS_VENV_DIR}/bin/python" -m colcon)
ROS_PACKAGES=(
  myarm_m750_description
  myarm_m750_driver
  myarm_m750_bringup
  myarm_m750_visualization
  myarm_m750_camera
  myarm_m750_moveit_config
)

if [[ ! -f "${ROS_SETUP_FILE}" || ! -x "${ROS_VENV_DIR}/bin/python" ]]; then
  echo "Missing ROS Foxy or .venv-ros. Run ./tools/bootstrap_ros.sh first." >&2
  exit 2
fi

ROS_GATE_DIR="$(mktemp -d "${TMPDIR:-/tmp}/myarm-m750-ros-gate.XXXXXXXX")"
ROS_BUILD_DIR="${ROS_GATE_DIR}/build"
ROS_INSTALL_DIR="${ROS_GATE_DIR}/install"
ROS_LOG_DIR="${ROS_GATE_DIR}/log"
trap 'rm -rf "${ROS_GATE_DIR}"' EXIT

unset PYTHONHOME PYTHONPATH
export PYTHONNOUSERSITE=1
# launch_testing must load its Foxy plugins inside the isolated ROS venv.
unset PYTEST_DISABLE_PLUGIN_AUTOLOAD || true

# shellcheck disable=SC1091
set +u
source "${ROS_SETUP_FILE}"
set -u
# shellcheck disable=SC1091
source "${ROS_VENV_DIR}/bin/activate"

PINOCCHIO_VERSION="$(
  "${ROS_VENV_DIR}/bin/python" -c \
    'import pinocchio; print(pinocchio.__version__)'
)"
if [[ "${PINOCCHIO_VERSION}" != "2.6.17" ]]; then
  echo "ROS gate requires Pinocchio 2.6.17; got ${PINOCCHIO_VERSION}." >&2
  exit 2
fi
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  "${ROS_VENV_DIR}/bin/python" -m pytest -q \
  "${ROOT_DIR}/pycore/tests/test_pinocchio_kinematics.py"

"${ROS_VENV_DIR}/bin/python" -m py_compile \
  "${ROOT_DIR}/tools/ros_runtime_gate.py" \
  "${ROOT_DIR}/tools/moveit_runtime_gate.py" \
  "${ROOT_DIR}/ros2/src/myarm_m750_moveit_config/test/moveit_runtime_probe.py"
"${ROS_VENV_DIR}/bin/python" -m flake8 \
  "${ROOT_DIR}/tools/ros_runtime_gate.py" \
  "${ROOT_DIR}/tools/moveit_runtime_gate.py" \
  "${ROOT_DIR}/ros2/src/myarm_m750_moveit_config/test/moveit_runtime_probe.py"

"${ROS_VENV_DIR}/bin/python" "${ROOT_DIR}/tools/model/generate_models.py" --check

"${ROS_COLCON[@]}" --log-base "${ROS_LOG_DIR}" build \
  --base-paths "${ROOT_DIR}/ros2/src" \
  --build-base "${ROS_BUILD_DIR}" \
  --install-base "${ROS_INSTALL_DIR}" \
  --symlink-install \
  --packages-select "${ROS_PACKAGES[@]}"

# shellcheck disable=SC1091
set +u
source "${ROS_INSTALL_DIR}/setup.bash"
set -u
"${ROS_COLCON[@]}" --log-base "${ROS_LOG_DIR}" test \
  --build-base "${ROS_BUILD_DIR}" \
  --install-base "${ROS_INSTALL_DIR}" \
  --packages-select "${ROS_PACKAGES[@]}" \
  --return-code-on-test-failure
"${ROS_COLCON[@]}" test-result \
  --test-result-base "${ROS_BUILD_DIR}" \
  --verbose

"${ROS_VENV_DIR}/bin/python" "${ROOT_DIR}/tools/ros_runtime_gate.py" \
  --core-config "${ROOT_DIR}/pycore/config/default.yaml" \
  --camera-config "${ROOT_DIR}/pycore/config/camera/cameras_mock.yaml" \
  --log-directory "${ROS_GATE_DIR}/runtime-logs"

"${ROS_VENV_DIR}/bin/python" "${ROOT_DIR}/tools/moveit_runtime_gate.py" \
  --core-config "${ROOT_DIR}/pycore/config/default.yaml" \
  --log-directory "${ROS_GATE_DIR}/moveit-runtime-logs"

echo "ROS Foxy source/headless gate passed."
