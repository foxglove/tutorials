#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
mkdir -p /recordings

if [[ "${1:-}" == "ros2" && "${2:-}" == "launch" && "${3:-}" == "intrinsic_foxglove_demo" && "${4:-}" == "demo.launch.py" ]]; then
  shift 4
  child=0
  stopping=0
  forward() {
    stopping=1
    if [[ "${child}" -ne 0 ]]; then
      # The launch process is a session leader. Signal the whole group so
      # rosbag2 sees SIGINT and writes metadata.yaml plus the MCAP summary.
      kill -INT -- "-${child}" 2>/dev/null || kill -INT "${child}" 2>/dev/null || true
    fi
  }
  trap forward INT TERM
  python3 -c 'import os, sys; os.setsid(); os.execvp(sys.argv[1], sys.argv[1:])' \
    ros2 launch intrinsic_foxglove_demo demo.launch.py \
    "record:=${RECORD:-true}" \
    "cycles:=${DEMO_CYCLES:-0}" \
    "$@" &
  child=$!
  signal_at=0
  set +e
  while true; do
    if ! kill -0 "${child}" 2>/dev/null; then
      break
    fi
    if [[ "${stopping}" -eq 1 ]]; then
      if [[ "${signal_at}" -eq 0 ]]; then
        signal_at=${SECONDS}
      fi
      if compgen -G "/recordings/intrinsic_grasp_demo_*/metadata.yaml" > /dev/null; then
        break
      fi
      if [[ $((SECONDS - signal_at)) -ge 25 ]]; then
        break
      fi
    fi
    sleep 0.3
  done
  if kill -0 "${child}" 2>/dev/null; then
    kill -TERM -- "-${child}" 2>/dev/null || true
    sleep 1
  fi
  wait "${child}"
  status=$?
  # Bag finalization can rewrite metadata.yaml as it exits. Chown after that.
  chown -R "$(stat -c '%u:%g' /recordings)" /recordings || true
  set -e
  exit "${status}"
fi

exec "$@"
