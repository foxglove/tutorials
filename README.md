# Foxglove Tutorials

This repository contains a collection of [Foxglove](https://foxglove.dev/) tutorials grouped by dataset and integration use cases.

Below is a list of all tutorials available in this repository:

## Datasets
### [Foxglove Visualization for 1X World Model Challenge](datasets/1x_eve/README.md)
- 📝 Description: Use Foxglove to visualize 1X World Model Challenge
### [Convert .mat files to MCAP](datasets/NASA_mat_to_MCAP/README.md)
- 📝 Description: Visualize Matlab files for NASA Valkyrie box pickup task
### [Converting Argoverse 2 Dataset to MCAP](datasets/foxglove_av2_tutorial/README.md)
- 📝 Description: This example converts the LiDAR scans from an AV2 log to MCAP.
- 🎥 [Video](https://youtu.be/tBj1LrL1v18)
### [Unitree LAFAN1 Retargeting Visualization](datasets/lafan1/README.md)
- 📝 Description: Retarget raw motion data to an Unitree G1 humanoid robot
- 🔗 [Related Blog Post](https://foxglove.dev/blog/converting-the-lafan1-retargeting-dataset-to-mcap)
- 🎥 [Video](https://youtu.be/YlAblmWLVqs)
- 📊 [Visualize](https://app.foxglove.dev/~/view?ds=foxglove-sample-stream&ds.recordingId=rec_0dVfPhEze7PkjHHi&layoutId=lay_0dVfPwEqAQ5JMmle)
### [SubPipe Dataset to MCAP](datasets/subpipe_mcap/README.md)
- 📝 Description: The underwater dataset, containing SLAM, object detection and image segmentation
- 🎥 [Video](https://youtu.be/jJej6aT1jKg)
- 📊 [Visualize](https://app.foxglove.dev/~/view?ds=foxglove-sample-stream&ds.recordingId=vqKKQcot421Kwg84&ds.overrideLayoutId=b7513959-1d46-4a89-bc24-1584d9677ca1&ds.start=2023-09-01T13:19:45.047438263Z&ds.end=2023-09-01T13:20:15.047438263Z)

## Foxglove SDK
### [Visualizing LeRobot (SO-100) using Foxglove](foxglove_sdk/foxglove_so_100/README.md)
- 📝 Description: Use LeRobot API and Foxglove SDK to visualize SO-100 state in real-time
- 🔗 [Related Blog Post](https://foxglove.dev/blog/visualizing-lerobot-so-100-using-foxglove)
### [ROSCON Spain 2024 MCAP Tutorial](foxglove_sdk/rosconesp24_tutorial/README.md)
- 📝 Description: Convert battery percentage, coordinates, and velocities to MCAP

## Integrations - ROS1
### [ROS 1 Actions Tutorial](integrations/ros1/actions/README.md)
- 📝 Description: Code reference for ROS 1 actions tutorial
- 🔗 [Related Blog Post](https://foxglove.dev/blog/creating-ros1-actions)
### [ROS1 Gazebo Simulation Demo](integrations/ros1/gazebo/README.md)
- 📝 Description: Code reference for ROS 1 Gazebo simulation tutorial
- 🔗 [Related Blog Post](https://foxglove.dev/blog/simulating-robotic-scenarios-with-ros1-and-gazebo)
### [ROS 1 Launch Files Tutorial](integrations/ros1/launch/README.md)
- 📝 Description: Code reference for ROS 1 launch files tutorial
- 🔗 [Related Blog Post](https://foxglove.dev/blog/how-to-use-ros1-launch-files)
### [ROS 1 Parameters Tutorial](integrations/ros1/params/README.md)
- 📝 Description: Code reference for ROS 1 parameters tutorial
- 🔗 [Related Blog Post](https://foxglove.dev/blog/how-to-use-ros1-parameters)
### [RGBA Point Cloud Tutorial](integrations/ros1/rgba-point-cloud/README.md)
- 📝 Description: Code reference for RGBA point cloud tutorial
- 🔗 [Related Blog Post](https://foxglove.dev/blog/visualizing-point-clouds-with-custom-colors)
### [Rosbridge Demo with ROS 1 and ROS 2](integrations/ros1/rosbridge/README.md)
- 📝 Description: Interactive demo showing how to use Rosbridge with ROS
- 🔗 [Related Blog Post](https://foxglove.dev/blog/using-rosbridge-with-ros1)
- 📊 [Visualize](https://foxglove.github.io/rosbridge-demo/)
### [ROS 1 Transforms Tutorial](integrations/ros1/transforms/README.md)
- 📝 Description: Code reference for ROS 1 transforms tutorial
- 🔗 [Related Blog Post](https://foxglove.dev/blog/publishing-and-visualizing-ros1-transforms)

## Integrations - ROS2
### [ROS 2 Actions Tutorial](integrations/ros2/actions/README.md)
- 📝 Description: Code reference for ROS 2 actions tutorial
- 🔗 [Related Blog Post](https://foxglove.dev/blog/creating-ros2-actions)
### [ROS 2 Diagnostics Tutorial](integrations/ros2/diagnostics/README.md)
- 📝 Description: Basic example of publishing and visualizing DiagnosticArray messages
- 🔗 [Related Blog Post](https://foxglove.dev/blog/a-practical-guide-to-using-ros-diagnostics)
### [ROS 2 Launch Files Tutorial](integrations/ros2/launch/README.md)
- 📝 Description: Code reference for ROS 2 launch files tutorial
- 🔗 [Related Blog Post](https://foxglove.dev/blog/how-to-use-ros2-launch-files)
### [ROS 2 Nav2 Tutorial](integrations/ros2/nav2/README.md)
- 📝 Description: Tutorial files for Navigation 2 with Foxglove
- 🔗 [Related Blog Post](https://foxglove.dev/blog/autonomous-robot-navigation-and-nav2-the-first-steps)
### [ROS 2 Parameters Tutorial](integrations/ros2/params/README.md)
- 📝 Description: Code reference for ROS 2 parameters tutorial
- 🔗 [Related Blog Post](https://foxglove.dev/blog/how-to-use-ros2-parameters)
### [Rosbridge Demo with ROS 1 and ROS 2](integrations/ros2/rosbridge/README.md)
- 📝 Description: Interactive demo showing how to use Rosbridge with ROS
- 🔗 [Related Blog Post](https://foxglove.dev/blog/using-rosbridge-with-ros2)
- 📊 [Visualize](https://foxglove.github.io/rosbridge-demo/)
### [ROS 2 Rust Tutorial (rclrs)](integrations/ros2/rust/README.md)
- 📝 Description: Tutorial for using ROS Client Library for Rust
- 🔗 [Related Blog Post](https://foxglove.dev/blog/first-steps-using-rust-with-ros2)

## Jupyter Notebooks
### [Analyze Your Robotics Data with Jupyter Notebooks](jupyter_notebooks/data_platform/README.md)
- 📝 Description: Load and analyze data in Jupyter Notebooks using Foxglove Data Management
- 🔗 [Related Blog Post](https://foxglove.dev/blog/analyze-your-robotics-data-with-jupyter-notebooks)

Join the Foxglove [Discord](https://discord.gg/UEuytgVkks) and follow [our blog](https://foxglove.dev/blog) for more ideas on how to integrate Foxglove into your robotics development workflows.