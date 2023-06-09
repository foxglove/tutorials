#!/bin/bash
set -e

if pgrep -x "caddy" > /dev/null; then
    echo "Caddy is already running or not installed."
else
    echo "Caddy is not running. Starting Caddy..."
    nohup caddy run --config ~/catkin_ws/src/.devcontainer/Caddyfile &
    echo "Caddy started."
fi