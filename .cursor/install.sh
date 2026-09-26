#!/usr/bin/env bash
# Cloud Agent install script for the Foxglove tutorials repository.
# Idempotent: safe to run repeatedly. Creates an isolated Python virtual
# environment and installs the dependencies needed to run the repo-wide
# README generator plus the runnable Foxglove SDK Python tutorials.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="${FOXGLOVE_TUTORIALS_VENV:-$HOME/.venvs/foxglove-tutorials}"

# System package required to create Python 3 virtual environments on Ubuntu.
if ! python3 -c "import ensurepip" >/dev/null 2>&1; then
  sudo apt-get update
  sudo apt-get install -y "python3-venv"
fi

# Create the virtual environment if it does not already exist.
if [ ! -x "$VENV_DIR/bin/python" ]; then
  python3 -m venv "$VENV_DIR"
fi

PY="$VENV_DIR/bin/python"
"$PY" -m pip install --upgrade pip

# Consolidated dependency set:
#   - .utils/requirements.txt: README generator (jinja2, pyyaml) enforced in CI
#   - foxglove_sdk/ethernet_ip_integration/requirements.txt: foxglove-sdk, cpppo, pylogix, numpy
#   - pandas + mcap: used by the ROSCON Spain 2024 CSV -> MCAP tutorial
PIP_ARGS=()
[ -f "$REPO_ROOT/.utils/requirements.txt" ] && PIP_ARGS+=(-r "$REPO_ROOT/.utils/requirements.txt")
[ -f "$REPO_ROOT/foxglove_sdk/ethernet_ip_integration/requirements.txt" ] && \
  PIP_ARGS+=(-r "$REPO_ROOT/foxglove_sdk/ethernet_ip_integration/requirements.txt")

"$PY" -m pip install "${PIP_ARGS[@]}" pandas mcap

echo "Foxglove tutorials environment ready. Activate with:"
echo "  source \"$VENV_DIR/bin/activate\""
