import h5py

import ros2_mcap_utils

filename = "mcap/generalized_humanoid/data/raw_wipe/10.h5"

h5 = h5py.File(filename, 'r')

print(h5.keys())
print(h5["front_cloud"].shape)
print(h5["front_color"].shape)
print(h5["front_depth"].shape)


num_steps = h5["front_cloud"].shape[0]
print(num_steps)
timestamp = {"sec": 0, "nsec": 0}
d_time_ns = int(1/10*1e9)

for step in range(num_steps):
    pc = h5["front_cloud"][step]

    pc_msg = ros2_mcap_utils.generate_pc_msgs(ptcld=pc, ts=timestamp["nsec"])
    ros2_mcap_utils.writer.write("point_cloud", ros2_mcap_utils.serialize_message(
        pc_msg), timestamp["nsec"])

    timestamp["nsec"] += d_time_ns

h5.close()
