#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

find "${ROOT_DIR}" -type d \( \
  -name __pycache__ -o \
  -name .pytest_cache -o \
  -name .mypy_cache -o \
  -name .ruff_cache -o \
  -name '*.egg-info' \
\) -prune -exec rm -rf {} +

find "${ROOT_DIR}" -type f \( -name '*.pyc' -o -name '*.pyo' \) -delete
rm -rf "${ROOT_DIR}/pycore/build" "${ROOT_DIR}/pycore/dist"
rm -f "${ROOT_DIR}/logs/"*.jsonl "${ROOT_DIR}/logs/"*.log

echo "Cleaned Python caches, package build metadata, and runtime logs."
