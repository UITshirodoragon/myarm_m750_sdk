#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

"${ROOT_DIR}/tools/test_core.sh"

if [[ "${MYARM_RUN_ROS_TESTS:-0}" == "1" ]]; then
  "${ROOT_DIR}/tools/test_ros.sh"
else
  echo "ROS gate not requested. Run with MYARM_RUN_ROS_TESTS=1 after bootstrap_ros.sh."
fi

echo "All requested quality gates passed."
