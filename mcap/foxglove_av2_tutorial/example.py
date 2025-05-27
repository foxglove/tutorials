import foxglove 
import struct

from glob import glob
from foxglove.schemas import (PackedElementField, 
                              PackedElementFieldNumericType, 
                              PointCloud, Pose, Vector3, Quaternion, Timestamp)
from pathlib import Path
from pyarrow import feather
from pandas import DataFrame

def convert_msg(data: DataFrame, timestamp_ns: int) -> PointCloud:
    sec  = timestamp_ns // int(1e9)
    nsec = timestamp_ns % int(1e9)

    rows = len(data)
    pt_stride = 12
    buffer = bytearray(rows * pt_stride)
    offset = 0
    for pt in data.loc[:].to_numpy():
        x, y, z, _, _, _ = pt
        struct.pack_into("<fff", buffer, offset, x, y, z)
        offset += pt_stride
    
    fields = [
        PackedElementField(name="x", offset=0, type=PackedElementFieldNumericType.Float32),
        PackedElementField(name="y", offset=4, type=PackedElementFieldNumericType.Float32),
        PackedElementField(name="z", offset=8, type=PackedElementFieldNumericType.Float32)
    ]

    return PointCloud(
                    timestamp    = Timestamp(sec=sec, nsec=nsec),
                    frame_id     = "base_link",
                    pose         = Pose(position=Vector3(), orientation=Quaternion()),
                    point_stride = pt_stride,
                    fields       = fields,
                    data         = bytes(buffer)
                )

DATASET_PATH = "/home/alp/data/datasets"
LOG_ID = "00a6ffc1-6ce9-3bc3-a060-6006e9893a1a"

with foxglove.open_mcap(f"{LOG_ID}-lidar.mcap"):
    fpaths = sorted(glob(f"{DATASET_PATH}/{LOG_ID}/sensors/lidar/*.feather"))
    for fpath in fpaths:
        with Path(fpath).open("rb") as fhandle:
            data = feather.read_feather(fhandle, memory_map=True) 
            timestamp_ns = int(Path(fpath).stem.split(".")[0])
            foxglove.log(
                "/LIDAR_TOP",
                convert_msg(data, timestamp_ns)
            )