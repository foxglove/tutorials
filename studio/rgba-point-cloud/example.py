import sys
import math
import struct
​
from mcap_ros1.writer import Writer
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2, PointField
from rospy import Time
​
fields = [
    PointField("x", 0, PointField.FLOAT32, 1),
    PointField("y", 4, PointField.FLOAT32, 1),
    PointField("z", 8, PointField.FLOAT32, 1),
    PointField("rgba", 12, PointField.UINT32, 1),
]
# Create a struct corresponding to the fields we defined above:
# One float each for x,y,z and one byte each for r,g,b,a
point_struct = struct.Struct("<fffBBBB")
​
def color(point: Point, t: float):
    r = 0.5 + 0.5 * point.x / 20
    g = point.y / 20
    b = 0.5 + 0.5 * math.sin(t)
    a = 0.5 + 0.5 * ((point.x / 20) * (point.y / 20))
    return int(r * 255), int(g * 255), int(b * 255), int(a * 255)
​
def make_point_cloud(stamp: Time):
    # Create a 20x20 grid of points that move around over time
    t = stamp.to_sec()
    points = [
        Point(x=x + math.cos(t + y / 5), y=y, z=0) for x in range(20) for y in range(20)
    ]
    buffer = bytearray(point_struct.size * len(points))
    for i, point in enumerate(points):
        r, g, b, a = color(point, t)
        point_struct.pack_into(
            buffer, i * point_struct.size, point.x, point.y, point.z, r, g, b, a
        )
    return PointCloud2(
        header=Header(frame_id="pc2", stamp=stamp),
        height=1,
        width=len(points),
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=point_struct.size,
        row_step=len(buffer),
        data=buffer,
    )
​
with open(sys.argv[1], "wb") as f:
    ros_writer = Writer(f)
    for i in range(0, 60 * 10):
        stamp = Time(i / 60)
        ros_writer.write_message("/pc2", make_point_cloud(stamp), stamp.to_nsec())
    ros_writer.finish()
