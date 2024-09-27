# pose_republisher

This node serves as a republisher for the `goal_pose` needed when using Foxglove in simulation.

`foxglove_bridge` does not publish sim-timed poses.

Run like this:

`ros2 run pose_republisher pose_sim_republish --ros-args -p use_sim_time:=true`

