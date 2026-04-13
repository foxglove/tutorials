#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
PLAYER_DIR="${ROOT_DIR}/external/foxglove_mcap_player"
PLAYER_WS="${PLAYER_DIR}/ros2_ws"

mkdir -p "${ROOT_DIR}/external"

if [[ ! -d "${PLAYER_DIR}/.git" ]]; then
  git clone --depth=1 https://github.com/botsandus/foxglove_mcap_player.git "${PLAYER_DIR}"
fi

if [[ ! -d "${PLAYER_WS}" ]]; then
  echo "Expected workspace missing: ${PLAYER_WS}"
  exit 1
fi

cd "${PLAYER_WS}"
colcon build --packages-select foxglove_mcap_player

echo "foxglove_mcap_player built at: ${PLAYER_WS}"
