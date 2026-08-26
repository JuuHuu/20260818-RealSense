#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_DIR="${PROJECT_DIR}/.venv"

if python3 -c "import ensurepip" >/dev/null 2>&1; then
  python3 -m venv "${ENV_DIR}"
else
  echo "python3-venv is unavailable; bootstrapping virtualenv inside the project."
  env -u PYTHONPATH python3 -m pip install --ignore-installed --upgrade --target "${PROJECT_DIR}/.venv-bootstrap" virtualenv
  env -u PYTHONPATH PYTHONPATH="${PROJECT_DIR}/.venv-bootstrap" python3 -m virtualenv --clear "${ENV_DIR}"
fi
env -u PYTHONPATH "${ENV_DIR}/bin/python" -m pip install --upgrade pip setuptools wheel
env -u PYTHONPATH "${ENV_DIR}/bin/python" -m pip install -e "${PROJECT_DIR}[test]"

echo "Setup complete. Activate with: source ${ENV_DIR}/bin/activate"
echo "Then check the camera with: rs-camera-info"
echo "For an ordinary RGB camera, calibrate with: rgb-checkerboard-calibrate --config config/rgb_checkerboard.yaml"
