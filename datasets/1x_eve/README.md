# Foxglove Visualization for [1X World Model Challenge](https://github.com/1x-technologies/1xgpt).

![1X EVE](1x_eve.gif)


1. Download the validation folder from [HuggingFace](https://huggingface.co/datasets/1x-technologies/worldmodel). The 
2. Execute the script `unpack_data.py`. 
3. Play the rosbag generated in `output/ros2/1x_eve`, with the following nodes:

    a.  `ros2 run robot_state_publisher robot_state_publisher models/eve_r3.urdf`
    
    b. `ros2 run joint_state_publisher joint_state_publisher --ros-args -p source_list:=["joints"]`

    c. `ros2 launch foxglove_bridge foxglove_bridge_launch.xml`

4. Open Foxglove, connect to a local port and load the layout [1X_EVE](foxglove_layouts/1X%20EVE.json).


