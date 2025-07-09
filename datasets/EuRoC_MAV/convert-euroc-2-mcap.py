# This script is used to convert the Euroc dataset to MCAP format
# Author: Carson Kohlbrenner
# Date: 2025-06-27


import foxglove
import argparse
import os
import csv
import cv2
import yaml
import numpy as np
from foxglove import Schema, Channel
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, Vector3
from rclpy.serialization import serialize_message
from pathlib import Path

def getImageMsg(img_path: str, timestamp: int, cam_num: int) -> Image:
    # Load as grayscale image data
    img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise ValueError(f"Could not load image: {img_path}")
    
    height, width = img.shape
    
    sec = int(timestamp // 1e9)
    nsec = int(timestamp % 1e9)

    # Fill in the image message data
    ros_image = Image()
    ros_image.header = Header()
    ros_image.header.stamp.sec = sec
    ros_image.header.stamp.nanosec = nsec
    ros_image.header.frame_id = "cam"+str(cam_num)
    ros_image.height = height
    ros_image.width = width
    ros_image.encoding = "mono8" #Stereo images
    ros_image.step = width  # For mono8, 1 byte per pixel
    ros_image.data = img.tobytes()

    return ros_image

def read_images(cam_directory, channel, cam_num):
    # Loop through the data.csv file and read in the image files
    with open(cam_directory + "/data.csv", "r") as csv_file:
        reader = csv.reader(csv_file)
        next(reader)  # Skip the first row with headers
        for row in reader:
            timestamp = int(row[0])
            image_name = row[1]
            image_path = os.path.join(cam_directory, "data", image_name)
            if not os.path.exists(image_path):
                print(f"Image {image_path} does not exist")
                continue

            # Convert image to ROS2 message and write to channel
            image_msg = getImageMsg(image_path, timestamp, cam_num)
            channel.log(serialize_message(image_msg), log_time=timestamp)

def read_imu(imu_data_path, imu_channel, imu_yaml_path):
    '''
    IMU data is in the format:
    timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]

    We will convert this to a custom IMU message that interfaces with sensor_msgs/msgs/Imu.
    This will be in the form:
     - header (header)
     - orientation (quaternion)
     - orientation covariance (float32[9])
     - angular velocity (vector3)
     - angular velocity covariance (float32[9])
     - linear acceleration (vector3)
     - linear acceleration covariance (float32[9])
    '''

    # Get the IMU config with covariance information
    with open(imu_yaml_path, "r") as imu_yaml_file:
        imu_yaml = yaml.load(imu_yaml_file, Loader=yaml.FullLoader)

    # Get the noise and bias parameters, see https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model for more details
    sample_sqr_dt = np.sqrt(1.0/float(imu_yaml["rate_hz"]))
    sigma_gd = imu_yaml["gyroscope_noise_density"]*sample_sqr_dt
    sigma_ad = imu_yaml["accelerometer_noise_density"]*sample_sqr_dt

    # Calculate the covariance matrices
    orientation_cov = np.zeros((3,3), dtype=np.float64)
    angular_velocity_cov = np.diag([sigma_gd**2, sigma_gd**2, sigma_gd**2]).astype(np.float64)
    linear_acceleration_cov = np.diag([sigma_ad**2, sigma_ad**2, sigma_ad**2]).astype(np.float64)

    with open(imu_data_path, "r") as imu_file:
        reader = csv.reader(imu_file)
        next(reader)  # Skip the first row with headers
        for row in reader:
            timestamp = int(row[0])
            angular_velocity = [float(row[1]), float(row[2]), float(row[3])]
            linear_acceleration = [float(row[4]), float(row[5]), float(row[6])]

            imu_msg = Imu()
            imu_msg.header = Header()
            imu_msg.header.stamp.sec = timestamp // int(1e9)
            imu_msg.header.stamp.nanosec = timestamp % int(1e9)
            imu_msg.header.frame_id = "imu4" # Transformation reference frame
            # Orientation
            imu_msg.orientation = Quaternion()
            imu_msg.orientation.x = 0.0
            imu_msg.orientation.y = 0.0
            imu_msg.orientation.z = 0.0
            imu_msg.orientation.w = 1.0
            imu_msg.orientation_covariance = list(orientation_cov.flatten(order="C").astype(float))
            # Angular velocity
            imu_msg.angular_velocity = Vector3()
            imu_msg.angular_velocity.x = angular_velocity[0]
            imu_msg.angular_velocity.y = angular_velocity[1]
            imu_msg.angular_velocity.z = angular_velocity[2]
            imu_msg.angular_velocity_covariance = list(angular_velocity_cov.flatten(order="C").astype(float))
            # Linear acceleration
            imu_msg.linear_acceleration = Vector3()
            imu_msg.linear_acceleration.x = linear_acceleration[0]
            imu_msg.linear_acceleration.y = linear_acceleration[1]
            imu_msg.linear_acceleration.z = linear_acceleration[2]
            imu_msg.linear_acceleration_covariance = list(linear_acceleration_cov.flatten(order="C").astype(float))

            imu_channel.log(serialize_message(imu_msg), log_time=timestamp)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Convert Euroc dataset to MCAP format')
    parser.add_argument('--src', type=str, required=True, help='Path to the Euroc dataset directory')
    parser.add_argument('--dst', type=str, required=True, help='Path to the output MCAP file')
    args = parser.parse_args()

    out_mcap_path = args.dst
    if os.path.exists(out_mcap_path): # Remove the previous file if it already exists
        os.remove(out_mcap_path)
    
    writer = foxglove.open_mcap(out_mcap_path, allow_overwrite=True) # Open the mcap file for writing

    # Define the schemas for our topics
    # get the .msg file that ROS installs
    # image_msg_path = Path(
    #     get_package_share_directory("sensor_msgs")) / "msg" / "Image.msg"
    # imu_msg_path = Path(
    #     get_package_share_directory("sensor_msgs")) / "msg" / "Imu.msg"
    imu_msg_path = Path("msgs/imu_flat.msg")
    img_msg_path = Path("msgs/image_flat.msg")

    img_schema = Schema(
        name="sensor_msgs/msg/Image",
        encoding="ros2msg",
        data=img_msg_path.read_bytes(),
    )
    imu_schema = Schema(
        name="sensor_msgs/msg/Imu",
        encoding="ros2msg",
        data=imu_msg_path.read_bytes(),
    )

    # ros2msg channels require cdr encoding type
    cam0_channel = Channel(topic="/cam0/image_raw", schema=img_schema, message_encoding="cdr")
    cam1_channel = Channel(topic="/cam1/image_raw", schema=img_schema, message_encoding="cdr")
    imu_channel = Channel(topic="/imu0", schema=imu_schema, message_encoding="cdr")

    # Read in the data.csv file
    data_csv_path = os.path.join(args.src, "data.csv")
    cam0_dir = os.path.join(args.src, "cam0")
    cam1_dir = os.path.join(args.src, "cam1")
    imu_dir = os.path.join(args.src, "imu0", "data.csv")
    imu_yaml_path = os.path.join(args.src, "imu0", "sensor.yaml")


    read_images(cam0_dir, cam0_channel, 0)
    print("Done writing cam0")
    read_images(cam1_dir, cam1_channel, 1)
    print("Done writing cam1")
    read_imu(imu_dir, imu_channel, imu_yaml_path)
    print("Done writing imu")
    