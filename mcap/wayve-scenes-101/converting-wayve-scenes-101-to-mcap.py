# WayveScenes101

# Import required libraries
import io
from PIL import Image
import pycolmap
from foxglove_schemas_protobuf.CompressedImage_pb2 import CompressedImage
from google.protobuf.timestamp_pb2 import Timestamp
from mcap_protobuf.writer import Writer

# Define paths
DATASET_PATH = "/path/to/wayve_scenes_101"
SCENE = "scene_001"
RECONSTRUCTION = "colmap_sparse/rig/"
IMAGES_FOLDER = "images"
MCAP_FILE = "wayve_scenes_101.mcap"

# Function to get image bytes
def getImage(img_path: str) -> bytes:
    with Image.open(f"{DATASET_PATH}/{SCENE}/{IMAGES_FOLDER}/{img_path}") as img:
        img_byte_array = io.BytesIO()
        img.save(img_byte_array, format="JPEG")
        img_bytes = img_byte_array.getvalue()
    return img_bytes

# Function to create a CompressedImage message
def getCompressedImgMsg(img: pycolmap.Image) -> CompressedImage:
    img_path = img.name

    # Extract timestamp from filename
    timestamp_micros = int(img_path.split('/')[-1].split('.')[0])
    seconds = timestamp_micros // int(1e6)
    nanos = (timestamp_micros - int(1e6) * seconds) * int(1e3)
    timestamp_ns = (seconds * int(1e9)) + (nanos // int(1e3))

    # Create and populate the CompressedImage message
    img_msg = CompressedImage()
    img_msg.timestamp.CopyFrom(Timestamp(seconds=seconds, nanos=nanos))
    img_msg.frame_id = f"camera-{img.camera_id}"
    img_msg.data = getImage(img_path)
    img_msg.format = "jpg"

    return img_msg, timestamp_ns

# Load the reconstruction
scene = pycolmap.Reconstruction(f"{DATASET_PATH}/{SCENE}/{RECONSTRUCTION}")

# Write to .mcap file
with open(MCAP_FILE, "wb") as f, Writer(f) as mcap_writer:
    for _, img in scene.images.items():
        # Generate the protobuf message
        img_msg, timestamp_ns = getCompressedImgMsg(img)

        # Write message to .mcap
        mcap_writer.write_message(
            topic="wayve/image_data",  # Use a meaningful topic name
            message=img_msg,
            log_time=timestamp_ns,
            publish_time=timestamp_ns,
        )
        print(f"Processed and wrote image: {img.name}")

print(f"Data successfully written to {MCAP_FILE}")