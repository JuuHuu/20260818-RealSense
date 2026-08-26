#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 "${PROJECT_DIR}/.venv/bin/python" -m pytest -q
