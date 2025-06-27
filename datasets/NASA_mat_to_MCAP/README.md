# Introduction

This folder contains an example on how to generate MCAP files from `.mat` files typically used in MATLAB or Octave.
In this case, each variable is treated individually, one MCAP file is generated for each variable, for the sake of the example. However, multiple channels can be written to the same MCAP file. Please see also [ROSCon Spain MCAP tutorial](https://github.com/foxglove/tutorials/tree/main/mcap/rosconesp24_tutorial).

# Download the dataset

1. The required file **is already included** in this repo, but if you want you can download the dataset from [NASA Valkyrie Box Pickup](https://repository.library.northeastern.edu/files/neu:m041tz63c).


# Generate CSV

2. Using [Octave](https://wiki.octave.org/GNU_Octave_Wiki), save `box_pickup_ihmc.mat` as CSV. The script `mat_to_csv.m` will load the `.mat` file and generate `.csv` files without headers.

3. Run the script `csv_header.py` to add the header to each CSV file.

# Generate MCAP

4. Generate one `.mcap` file per `.csv` file using the scripts `csv_to_mcap.py` or `csv_to_ros2_mcap.py` for ROS 2 type messages.

# Merge the MCAP files

5. Using `ros2 bag convert` tool, merge all the `.mcap` files into one with the help of the script `rosbag.bash` and the configuration file `merge_mcap.yaml`.

`ros2 bag convert $(bash rosbag.bash) -o merge_mcap.yaml`