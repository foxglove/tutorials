---
title: "ROS 2 Jazzy TurtleBot3 Diff-Drive Tuning with Pixi"
short_description: "Headless Gazebo simulation, MCAP recording, and live wheel radius/separation modifier tuning with Foxglove"
---

# ROS 2 Jazzy + Pixi: TurtleBot3 diff-drive tuning workflow

This tutorial creates an end-to-end, **pixi-only** workflow for:

1. Launching a **headless Gazebo** simulation with TurtleBot3 (Burger) on ROS 2 Jazzy
2. Recording an MCAP bag from the simulation
3. Replaying that bag with [`foxglove_mcap_player`](https://github.com/botsandus/foxglove_mcap_player)
4. Tuning diff-drive wheel calibration live from Foxglove by updating ROS parameters

## What this tutorial contains

- `pixi.toml`: ROS 2 Jazzy environment and repeatable tasks
- `launch/headless_turtlebot3_gazebo.launch.py`: Gazebo server-only launch (no GUI)
- `scripts/diff_drive_calibrator.py`: runtime diff-drive wheel modifier calibrator node
- `config/diff_drive_calibrator_sim.yaml`: calibration params for simulation (`/cmd_vel_raw` -> `/cmd_vel`)
- `config/diff_drive_calibrator_replay.yaml`: calibration params for replay (`/cmd_vel` -> `/cmd_vel_tuned`)
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

## 3) Start the diff-drive wheel calibration node

Terminal B:

```bash
pixi run calibrate-sim
```

The calibrator subscribes to `/cmd_vel_raw` and publishes compensated commands to `/cmd_vel`.
Tuneable parameters are namespaced under `/diff_drive_calibrator`.

The main live tuning parameters are:

- `wheel_radius_multiplier`
- `wheel_separation_multiplier`

These map to the diff-drive calibration concept directly:
- if wheel radius is effectively too small/large, tune `wheel_radius_multiplier`
- if turning radius is off, tune `wheel_separation_multiplier`

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

## 7) Tune diff-drive wheel modifiers during replay in Foxglove

Terminal F:

```bash
pixi run calibrate-replay
```

Replay mode subscribes to `/cmd_vel` and publishes compensated commands to `/cmd_vel_tuned`.
This keeps original and calibrated command streams separate for comparison.

In Foxglove, add panels for:

- `/cmd_vel` (original)
- `/cmd_vel_tuned` (after calibration)
- `/odom` and `/tf`

Adjust ROS parameters on `/diff_drive_calibrator` live:

- `wheel_radius_multiplier`
- `wheel_separation_multiplier`
- `cmd_timeout`

## Notes

- Default robot model is `burger` via pixi activation env (`TURTLEBOT3_MODEL`).
- If your bag contains a different file name than `tb3_0.mcap`, point `MCAP_FILE` to the actual path.
- `foxglove_mcap_player` is not currently packaged in RoboStack Jazzy, so this tutorial builds it from source in `external/`.
- TurtleBot3 Jazzy in Gazebo uses `gz::sim::systems::DiffDrive` with static SDF wheel params. The calibrator node keeps tuning live by exposing ROS parameters and pre-compensating `cmd_vel` in real time.
