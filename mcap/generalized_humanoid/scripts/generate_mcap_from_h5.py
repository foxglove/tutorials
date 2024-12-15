import os
import h5py
import argparse

import ros2_mcap_utils
import mcap_utils


class H5McapGenerator():
    def __init__(self, h5_filename="mcap/generalized_humanoid/data/raw_wipe/10.h5"):
        filename = h5_filename

        rosbag_name = os.path.basename(os.path.dirname(
            filename))+'_'+os.path.basename(filename).replace(".h5", "")

        h5 = h5py.File(filename, 'r')

        print(f"Generating MCAP {rosbag_name} from H5 file {filename}")

        num_steps = h5["front_cloud"].shape[0]
        timestamp = {"sec": 0, "nsec": 0}
        d_time_ns = int(1/10*1e9)

        topics_and_types = {
            "joints": "JointState",
            "point_cloud": "PointCloud2",
            "camera": "CompressedImage",
            "camera/depth": "Image"
        }

        mcap_writer = mcap_utils.McapWriter(bag_name=rosbag_name)
        ros2_writer = ros2_mcap_utils.Ros2Writer(bag_name=rosbag_name)

        mcap_writer.create_channels(topics_and_types)
        ros2_writer.create_topics(topics_and_types)

        for step in range(num_steps):
            # Color image
            img = h5["front_color"][step]
            img_mcap_msg = mcap_utils.generate_comp_camera_msgs(
                img=img, ts=timestamp["nsec"])
            mcap_writer.write_message(
                "camera", img_mcap_msg, timestamp["nsec"])
            img_msg = ros2_mcap_utils.generate_comp_camera_msgs(
                img=img, ts=timestamp["nsec"])
            ros2_writer.write_topic("camera", img_msg, timestamp["nsec"])

            # Depth image
            depth_img = h5["front_depth"][step]
            depth_mcap_img = mcap_utils.generate_raw_image_msgs(
                img=depth_img.copy(), ts=timestamp["nsec"])
            mcap_writer.write_message(
                "camera/depth", depth_mcap_img, timestamp["nsec"])
            depth_img_msg = ros2_mcap_utils.generate_image_msgs(
                img=depth_img.copy(), ts=timestamp["nsec"])
            ros2_writer.write_topic(
                "camera/depth", depth_img_msg, timestamp["nsec"])

            # PointCloud
            pc = h5["front_cloud"][step]
            pc_mcap_msg = mcap_utils.generate_pc_msgs(
                ptcld=pc, ts=timestamp["nsec"])
            mcap_writer.write_message(
                "point_cloud", pc_mcap_msg, timestamp["nsec"])
            pc_msg = ros2_mcap_utils.generate_pc_msgs(
                ptcld=pc, ts=timestamp["nsec"])
            ros2_writer.write_topic("point_cloud", pc_msg, timestamp["nsec"])

            # Joints
            joints = h5["qpos_action"][step]
            joints32 = ros2_mcap_utils.joints_util.joint25_to_joint32(joints)
            joint_msg = ros2_mcap_utils.generate_joint_msgs(
                joints32, timestamp["nsec"])
            ros2_writer.write_topic("joints", joint_msg, timestamp["nsec"])

            timestamp["nsec"] += d_time_ns

            print(f"{step}/{num_steps}")

        h5.close()
        mcap_writer.close()


parser = argparse.ArgumentParser()
parser.add_argument("--h5_dataset_path", type=str,
                    default="data/raw_wipe/10.h5")

args = parser.parse_args()
h5_dataset_path = args.h5_dataset_path

H5McapGenerator(h5_dataset_path)
