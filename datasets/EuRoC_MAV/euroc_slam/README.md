# Custom ROS2 launch files with drone URDF
These files are intended to be copied into a ROS2 package to launch the full EuRoC visualization demo! The urdf and meshes used for this vizualization are ported from [RotorS](https://github.com/ethz-asl/rotors_simulator) to work with ROS2. Please refer to ["RotorS---A Modular Gazebo MAV Simulator Framework"](https://link.springer.com/chapter/10.1007/978-3-319-26054-9_23) to learn more. It is assumed you already have a working instillation of ROS2. This repo was tested on ROS2 Humble. Make sure you have already installed the prerequisites found [here](../README.md)

![RotorS drone](../imgs/close.png)

## First, make a new package
```bash
cd ros2_ws/src
ros2 pkg create --build-type ament_python --license Apache-2.0 euroc_slam
cd euroc_slam
```
Next, copy the contents of this folder to your new package
```bash
cp -a ~/euroc-2-mcap/euroc_slam/. ~/ros2_ws/src/euroc_slam
```
Configure the setup.py file to see the models and launch files
```python
import os #Add this
from glob import glob # Add this
from setuptools import find_packages, setup

... #Add these
(os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
(os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
(os.path.join('share', package_name, 'meshes'), glob('meshes/*')),

... # Modify the entry points
entry_points={
    'console_scripts': [
        'firefly_state_publisher = euroc_slam.firefly_state_publisher:main',
    ],
},
```
Finally, build the package
```bash
cd ~/ros2_ws && colcon build --packages-select euroc_slam
```
## Give it a run!
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch euroc_slam firefly.launch.py
```
In a seperate terminal, play the data as an mcap
```bash
ros2 bag play your-data.mcap --clock -r 1
```