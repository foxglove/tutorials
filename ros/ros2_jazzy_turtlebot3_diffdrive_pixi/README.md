---
title: "ROS 2 Jazzy TurtleBot3 Diff-Drive Tuning with Pixi"
short_description: "Headless Gazebo simulation, MCAP recording, and live diff-drive tuning with Foxglove"
---

# ROS 2 Jazzy + Pixi: TurtleBot3 diff-drive tuning workflow

This tutorial creates an end-to-end, **pixi-only** workflow for:

1. Launching a **headless Gazebo** simulation with TurtleBot3 (Burger) on ROS 2 Jazzy
2. Recording an MCAP bag from the simulation
3. Replaying that bag with [`foxglove_mcap_player`](https://github.com/botsandus/foxglove_mcap_player)
4. Tuning diff-drive behavior live from Foxglove by updating ROS parameters

## What this tutorial contains

- `pixi.toml`: ROS 2 Jazzy environment and repeatable tasks
- `launch/headless_turtlebot3_gazebo.launch.py`: Gazebo server-only launch (no GUI)
- `scripts/diff_drive_tuner.py`: runtime diff-drive filter and tuner node
- `config/diff_drive_tuner_sim.yaml`: tuning params for simulation (`/cmd_vel_raw` -> `/cmd_vel`)
- `config/diff_drive_tuner_replay.yaml`: tuning params for replay (`/cmd_vel` -> `/cmd_vel_tuned`)
- `scripts/setup_foxglove_mcap_player.sh`: clones/builds `foxglove_mcap_player`
- `scripts/run_mcap_player.sh`: runs player against a chosen MCAP file

## 1) Enter the tutorial directory and install dependencies

```bash
cd ros/ros2_jazzy_turtlebot3_diffdrive_pixi
pixi install
```

The pixi project uses RoboStack Jazzy packages from:

- `https://prefix.dev/robostack-jazzy`
- `https://prefix.dev/conda-forge`

## 2) Start headless TurtleBot3 Gazebo sim

Terminal A:

```bash
pixi run sim
```

This launches Gazebo in server mode (`-s`) with no GUI client.

## 3) Start the diff-drive tuner for simulation

Terminal B:

```bash
pixi run tune-sim
```

The tuner subscribes to `/cmd_vel_raw` and publishes tuned velocity commands to `/cmd_vel`.
Tuneable parameters are namespaced under `/diff_drive_tuner`.

## 4) Drive the robot and record MCAP

Terminal C (drive):

```bash
pixi run teleop
```

Or publish a continuous circle command:

```bash
pixi run drive-circle
```

Terminal D (record):

```bash
pixi run bag-record
```

This creates `bags/tb3_<timestamp>/` with MCAP storage and captures:

- `/clock`
- `/cmd_vel_raw`
- `/cmd_vel`
- `/odom`
- `/tf`, `/tf_static`
- `/joint_states`
- `/scan`

Stop recording with `Ctrl+C`.

## 5) Start Foxglove bridge (optional live graph introspection)

Terminal E:

```bash
pixi run foxglove-bridge
```

Open Foxglove and connect to `ws://<machine-ip>:8765`.

## 6) Build and run foxglove_mcap_player for replay

First-time setup (clones and builds the player):

```bash
pixi run player-setup
```

Replay a bag with Foxglove websocket server + ROS republish:

```bash
MCAP_FILE="$(ls -d bags/tb3_* | tail -n 1)/tb3_0.mcap" pixi run mcap-play
```

You can set `PLAYER_HOST` and `PLAYER_PORT` if needed:

```bash
PLAYER_HOST=0.0.0.0 PLAYER_PORT=8766 MCAP_FILE=... pixi run mcap-play
```

## 7) Tune diff-drive during replay in Foxglove

Terminal F:

```bash
pixi run tune-replay
```

Replay mode subscribes to `/cmd_vel` and publishes tuned commands to `/cmd_vel_tuned`.
This keeps original and tuned command streams separate for comparison.

In Foxglove, add panels for:

- `/cmd_vel` (original)
- `/cmd_vel_tuned` (after tuning)
- `/odom` and `/tf`

Adjust ROS parameters on `/diff_drive_tuner` live:

- `linear_scale`
- `angular_scale`
- `max_linear_speed`
- `max_angular_speed`
- `linear_accel_limit`
- `angular_accel_limit`
- `cmd_timeout`

## Notes

- Default robot model is `burger` via pixi activation env (`TURTLEBOT3_MODEL`).
- If your bag contains a different file name than `tb3_0.mcap`, point `MCAP_FILE` to the actual path.
- `foxglove_mcap_player` is not currently packaged in RoboStack Jazzy, so this tutorial builds it from source in `external/`.
