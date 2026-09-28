#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
mkdir -p /recordings

if [[ "${1:-}" == "ros2" && "${2:-}" == "launch" && "${3:-}" == "intrinsic_foxglove_demo" && "${4:-}" == "demo.launch.py" ]]; then
  shift 4
  child=0
  forward() {
    if [[ "${child}" -ne 0 ]]; then
      # The launch process is a session leader. Signal the whole group so
      # rosbag2 sees SIGINT and writes metadata.yaml plus the MCAP summary.
      kill -INT -- "-${child}" 2>/dev/null || kill -INT "${child}" 2>/dev/null || true
    fi
  }
  trap forward INT TERM
  # Bash ignores SIGINT in asynchronous children. Reset it after setsid so
  # ros2 launch and rosbag2 still shut down and finalize the MCAP.
  python3 -c 'import os, signal, sys; os.setsid(); signal.signal(signal.SIGINT, signal.SIG_DFL); signal.signal(signal.SIGQUIT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])' \
    ros2 launch intrinsic_foxglove_demo demo.launch.py \
    "record:=${RECORD:-true}" \
    "cycles:=${DEMO_CYCLES:-0}" \
    "$@" &
  child=$!
  # wait returns early when the trap runs. Keep waiting until launch exits so
  # rosbag2 can finish the MCAP summary. stop_grace_period is the hard limit.
  set +e
  status=0
  while kill -0 "${child}" 2>/dev/null; do
    wait "${child}"
    status=$?
  done
  # Bag finalization can rewrite metadata.yaml as it exits. Chown after that.
  chown -R "$(stat -c '%u:%g' /recordings)" /recordings || true
  exit "${status}"
fi

exec "$@"
