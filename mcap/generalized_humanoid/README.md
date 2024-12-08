# Foxglove visualization for Generalizable Humanoid Manipulation

![Camera and PointCloud](camera_pc_humanoid.gif)

## Download the dataset

[Dataset link](https://humanoid-manipulation.github.io/)

[Download link](https://drive.google.com/drive/folders/1f5Ln_d14OQ5eSjPDGnD7T4KQpacMhgCB)

There are two types of files:

* h5: raw_wipe, raw_pour, etc...
* binary (zarr): training_data_example

## Binary data
* The dataset *training_data_example* was used in this visualization. The dataset is stored in `zarr` format. Install the Python module with `pip install zarr`.
* Color images are read from dataset in element `data/img`. Pointclouds are in element `data/point_cloud`, containing **10000** points with **6** fields: [x,y,z,r,g,b]

### Convert to MCAP

Generate a MCAP file using the script `python3 generate_mcap.py --dataset-path <path_to_dataset>`. This will create an output folder and `.mcap` files for each episode in the dataset.
These MCAP files contain:

* Color camera stream
* PointCloud

### MCAP ROS 2 Serialization

ROS 2 uses `cdr` serialization for messages, therefore standard MCAP files are not compatible with ROS 2. To generate a ROS 2 compatible MCAP file, use the script `python3 generate_mcap.py --dataset-path <path_to_dataset>`. This will create new MCAP files that can be reproduced visualized with Foxglove and reproduced with `ros2 bag play`.
These MCAP files contain:

* Color camera stream
* PointCloud
WIP:
* Robot state and joints

## H5 data

* The datasets *raw_wipe* and *raw_pour* were used in this visualization. The dataset is stored in `h5` format. Install the Python module with `pip install h5py`.

In this case, the script `generate_mcap_from_h5.py` generates two MCAP files simultanously, one is ROS 2 compatible and the other is MCAP.

This is an example of the use of the different libraries for MCAP and ROS 2 bags.


# Visualize in Foxglove

Import the layout [Humanoid.json](foxglove_layouts/Humanoid.json) in Foxglove. Then open a MCAP file.


