# Foxglove visualization for Generalizable Humanoid Manipulation

[Dataset link](https://humanoid-manipulation.github.io/)

# Download the dataset

* The dataset *training_data_example* was used in this visualization.
* The dataset is stored in `zarr` format. Install the Python module with `pip install zarr`.
* Images are easily read from dataset in element `data/img`.
* Pointclouds are in element `data/point_cloud`. Contains **10000** points with **6** fields: [x,y,z,r,g,b]

# Convert to MCAP

Generate a MCAP file using the script `python3 generate_mcap.py --dataset-path <path_to_dataset>`. This will create an output folder and `.mcap` files for each episode in the dataset.
These MCAP files contain:

* Images
* PointCloud


## MCAP ROS 2 Serialization

ROS 2 uses `cdr` serialization for messages, therefore standard MCAP files are not compatible with ROS 2. To generate a ROS 2 compatible MCAP file, use the script `python3 generate_mcap.py --dataset-path <path_to_dataset>`. This will create new MCAP files that can be reproduced visualized with Foxglove and reproduced with `ros2 bag play`.
These MCAP files contain:

* Images
* PointCloud
WIP:
* Robot state and joints

# Visualize in Foxglove

Import the layout [Humanoid.json](foxglove_layouts/Humanoid.json) in Foxglove. Then open a MCAP file.


![Camera and PointCloud](camera_pc_humanoid.gif)
