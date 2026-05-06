#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PLAYER_REPO_DIR="${PROJECT_ROOT}/external/foxglove_mcap_player"
PLAYER_WORKSPACE_DIR="${PLAYER_REPO_DIR}/ros2_ws"

MCAP_FILE="${MCAP_FILE:-}"
PORT="${PLAYER_PORT:-8766}"
HOST="${PLAYER_HOST:-127.0.0.1}"

if [[ $# -ge 1 ]]; then
  MCAP_FILE="$1"
fi
if [[ $# -ge 2 ]]; then
  PORT="$2"
fi
if [[ $# -ge 3 ]]; then
  HOST="$3"
fi

if [[ -z "${MCAP_FILE}" ]]; then
  echo "Usage: $0 <path-to-recording.mcap> [port] [host]"
  echo "Or set MCAP_FILE (and optional PLAYER_PORT/PLAYER_HOST) in the environment."
  exit 1
fi

if [[ ! -f "${MCAP_FILE}" ]]; then
  echo "MCAP file not found: ${MCAP_FILE}"
  exit 1
fi

if [[ ! -d "${PLAYER_WORKSPACE_DIR}" ]]; then
  echo "foxglove_mcap_player workspace not found. Run: pixi run player-setup"
  exit 1
fi

source "${PLAYER_WORKSPACE_DIR}/install/setup.bash"

ros2 run foxglove_mcap_player foxglove_mcap_player --ros-args \
  -p file:="${MCAP_FILE}" \
  -p port:="${PORT}" \
  -p host:="${HOST}"
