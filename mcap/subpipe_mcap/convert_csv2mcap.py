import typing
from pathlib import Path
import os
import csv
import base64
import json
import cv2

import numpy as np

from mcap.writer import Writer
from mcap.well_known import SchemaEncoding, MessageEncoding

# Define paths
ROOT_PATH = os.path.dirname(os.path.abspath(__file__))
PATH_TO_CSV_FILE = Path(
    os.path.join(ROOT_PATH, "DATA", "EstimatedState.csv"))
PATH_TO_IMAGES_FOLDER = Path(
    os.path.join(ROOT_PATH, "DATA", "Cam0_images"))
PATH_TO_GRAY_FOLDER = Path(
    os.path.join(ROOT_PATH, "DATA", "Cam1_images"))
PATH_TO_SEGMENT_FOLDER = Path(
    os.path.join(ROOT_PATH, "DATA", "Segmentation"))
PATH_TO_HF_FOLDER = Path(
    os.path.join(ROOT_PATH, "DATA", "SSS_HF_images"))
PATH_TO_LF_FOLDER = Path(
    os.path.join(ROOT_PATH, "DATA", "SSS_LF_images"))
PATH_TO_OUTPUT_FOLDER = Path(
    os.path.join(ROOT_PATH, "output"))
PATH_TO_VIDEO_FOLDER = Path(
    os.path.join(ROOT_PATH, "video"))

# Create output paths
if not os.path.exists(PATH_TO_OUTPUT_FOLDER):
    os.mkdir(PATH_TO_OUTPUT_FOLDER)

if not os.path.exists(PATH_TO_VIDEO_FOLDER):
    os.mkdir(PATH_TO_VIDEO_FOLDER)


def data_reader(csv_path: typing.Union[str, Path]):
    """ Reads data from a CSV file with predefined header """
    with open(csv_path, "r", encoding="utf-8") as f:
        for image_path, timestamp, lat, lon, est_depth, _, _, _, _, _, _, _, _, press, _, _, _, _, _, _, q, r, depth, alt in csv.reader(f):
            # Discard first row
            if image_path == "image":
                print("Header row")
                continue

            image = os.path.basename(image_path)
            yield (timestamp, image, float(lat), float(lon), float(est_depth), float(press), float(q), float(r), float(depth), float(alt))


def add_location_press_depth():
    locations = []
    press_depth = []

    for timestamp, image, lat, lon, est_depth, press, q, r, depth, alt in data_reader(PATH_TO_CSV_FILE):
        press_depth_val = {
            "x": press,
            "y": depth,
        }
        press_depth.append(press_depth_val)

        loc_val = {
            "timestamp": timestamp,
            "frame_id": "LAUV",
            "latitude": lat,
            "longitude": lon,
            "altitude": alt,
            "position_covariance": list(np.zeros(9)),
            "position_covariance_type": 0,
        }
        locations.append(loc_val)

    assert len(locations) == len(press_depth)

    with open(os.path.join(PATH_TO_OUTPUT_FOLDER, "subpipe.mcap"), "wb") as f:
        writer = Writer(f)
        writer.start("x-jsonschema")

        with open(Path(__file__).parent / "LocationFix.json", "rb") as f:
            schema = f.read()
            location_schema_id = writer.register_schema(
                name="foxglove.LocationFix",
                encoding=SchemaEncoding.JSONSchema,
                data=schema)
        with open(Path(__file__).parent / "Vector2.json", "rb") as f:
            schema = f.read()
            pressure_schema_id = writer.register_schema(
                name="foxglove.Vector2",
                encoding=SchemaEncoding.JSONSchema,
                data=schema)

        location_channel_id = writer.register_channel(
            topic="location_fix",
            message_encoding=MessageEncoding.JSON,
            schema_id=location_schema_id)
        pressure_channel_id = writer.register_channel(
            topic="pressure_depth",
            message_encoding=MessageEncoding.JSON,
            schema_id=pressure_schema_id)

        last_print = 0
        for i, (loc, pres) in enumerate(zip(locations, press_depth)):
            writer.add_message(
                location_channel_id,
                log_time=int(float(loc["timestamp"])*1e9),
                data=json.dumps(loc).encode("utf-8"),
                publish_time=int(float(loc["timestamp"])*1e9))
            writer.add_message(
                pressure_channel_id,
                log_time=int(float(loc["timestamp"])*1e9),
                data=json.dumps(pres).encode("utf-8"),
                publish_time=int(float(loc["timestamp"])*1e9))

            current_percentage = round(i/len(locations)*100, 2)
            if current_percentage-last_print > 5:
                print(f"{current_percentage} %")
                last_print = current_percentage

        writer.finish()


def add_gopro():
    """ Read GoPro images and  """
    timestamps = []
    images_path = []
    for timestamp, image, _, _, _, _, _, _, _, _ in data_reader(PATH_TO_CSV_FILE):
        timestamps.append(timestamp)
        images_path.append(os.path.join(PATH_TO_IMAGES_FOLDER, image))
    last_print = 0
    print(f"{len(images_path)} images")
    # images_path = images_path[0:2000]
    with open(os.path.join(PATH_TO_OUTPUT_FOLDER, "gopro.mcap"), "wb") as f:
        writer = Writer(f)
        writer.start("x-jsonschema")
        with open(Path(__file__).parent / "CompressedImage.json", "rb") as schema_f:
            schema = schema_f.read()
        camera_schema_id = writer.register_schema(
            name="foxglove.CompressedImage",
            encoding=SchemaEncoding.JSONSchema,
            data=schema)
        camera_channel_id = writer.register_channel(
            topic="cam0",
            message_encoding=MessageEncoding.JSON,
            schema_id=camera_schema_id)

        for i, (image_path, timestamp) in enumerate(zip(images_path, timestamps)):
            with open(image_path, "rb") as file:
                img = file.read()
            image_val = {
                "timestamp": timestamp,
                "frame_id": "gopro_link",
                "data": base64.b64encode(img).decode('utf-8'),
                "format": "jpg"
            }
            writer.add_message(
                camera_channel_id,
                log_time=int(float(image_val["timestamp"])*1e9),
                data=json.dumps(image_val).encode("utf-8"),
                publish_time=int(float(image_val["timestamp"])*1e9))

            current_percentage = round(i/len(images_path)*100, 2)
            if current_percentage-last_print > 5:
                print(f"Imgs: {current_percentage} %")
                last_print = current_percentage

        writer.finish()


def add_gray():
    images_path = [os.path.join(PATH_TO_GRAY_FOLDER, file)
                   for file in os.listdir(PATH_TO_GRAY_FOLDER)]
    with open(os.path.join(PATH_TO_OUTPUT_FOLDER, "gray.mcap"), "wb") as f:
        writer = Writer(f)
        writer.start("x-jsonschema")
        with open(Path(__file__).parent / "CompressedImage.json", "rb") as schema_f:
            schema = schema_f.read()
        camera_schema_id = writer.register_schema(
            name="foxglove.CompressedImage",
            encoding=SchemaEncoding.JSONSchema,
            data=schema)
        camera_channel_id = writer.register_channel(
            topic="cam1",
            message_encoding=MessageEncoding.JSON,
            schema_id=camera_schema_id)
        last_print = 0
        for i, image_path in enumerate(images_path):
            with open(image_path, "rb") as file:
                img = file.read()
            timestamp = image_path.split('/')[-1]
            timestamp = timestamp.replace('.jpg', '')
            image_val = {
                "timestamp": timestamp,
                "frame_id": "gray_link",
                "data": base64.b64encode(img).decode('utf-8'),
                "format": "jpg"
            }
            writer.add_message(
                camera_channel_id,
                log_time=int(float(image_val["timestamp"])*1e9),
                data=json.dumps(image_val).encode("utf-8"),
                publish_time=int(float(image_val["timestamp"])*1e9))

            current_percentage = round(i/len(images_path)*100, 2)
            if current_percentage-last_print > 5:
                print(f"Imgs: {current_percentage} %")
                last_print = current_percentage

        writer.finish()


def add_segmentation():
    images_path = [os.path.join(PATH_TO_SEGMENT_FOLDER, file)
                   for file in os.listdir(PATH_TO_SEGMENT_FOLDER)]
    with open(os.path.join(PATH_TO_OUTPUT_FOLDER, "segment.mcap"), "wb") as f:
        writer = Writer(f)
        writer.start("x-jsonschema")
        with open(Path(__file__).parent / "CompressedImage.json", "rb") as schema_f:
            schema = schema_f.read()
        camera_schema_id = writer.register_schema(
            name="foxglove.CompressedImage",
            encoding=SchemaEncoding.JSONSchema,
            data=schema)
        camera_channel_id = writer.register_channel(
            topic="cam0/segmentation",
            message_encoding=MessageEncoding.JSON,
            schema_id=camera_schema_id)
        label_channel_id = writer.register_channel(
            topic="cam0/label",
            message_encoding=MessageEncoding.JSON,
            schema_id=camera_schema_id)
        last_print = 0
        for i, image_path in enumerate(images_path):
            with open(image_path, "rb") as file:
                img = file.read()
            timestamp = image_path.split('/')[-1]
            timestamp = timestamp.replace('.png', '').replace('_label', '')
            if "_label" not in image_path:
                image_val = {
                    "timestamp": timestamp,
                    "frame_id": "LUAV",
                    "data": base64.b64encode(img).decode('utf-8'),
                    "format": "png"
                }
                writer.add_message(
                    camera_channel_id,
                    log_time=int(float(image_val["timestamp"])*1e9),
                    data=json.dumps(image_val).encode("utf-8"),
                    publish_time=int(float(image_val["timestamp"])*1e9))
            else:
                image_val = {
                    "timestamp": timestamp,
                    "frame_id": "LUAV",
                    "data": base64.b64encode(img).decode('utf-8'),
                    "format": "png"
                }
                writer.add_message(
                    label_channel_id,
                    log_time=int(float(image_val["timestamp"])*1e9),
                    data=json.dumps(image_val).encode("utf-8"),
                    publish_time=int(float(image_val["timestamp"])*1e9))

            current_percentage = round(i/len(images_path)*100, 2)
            if current_percentage-last_print > 5:
                print(f"Imgs: {current_percentage} %")
                last_print = current_percentage

        writer.finish()


def add_SSS_images(base_folder=PATH_TO_HF_FOLDER, convert=False, output="sss_hf", topic="sss/hf"):
    png_path = os.path.join(base_folder, "png")
    if convert:
        for img_file in os.listdir(base_folder):
            if img_file == "png":
                continue
            img = cv2.imread(os.path.join(base_folder, img_file))
            cv2.imwrite(os.path.join(
                png_path, img_file.replace('.pbm', '.png')), img)

    images_path = [os.path.join(png_path, file)
                   for file in os.listdir(png_path)]
    with open(os.path.join(PATH_TO_OUTPUT_FOLDER, output+".mcap"), "wb") as f:
        writer = Writer(f)
        writer.start("x-jsonschema")
        with open(Path(__file__).parent / "CompressedImage.json", "rb") as schema_f:
            schema = schema_f.read()
        camera_schema_id = writer.register_schema(
            name="foxglove.CompressedImage",
            encoding=SchemaEncoding.JSONSchema,
            data=schema)
        camera_channel_id = writer.register_channel(
            topic=topic,
            message_encoding=MessageEncoding.JSON,
            schema_id=camera_schema_id)

        last_print = 0
        for i, image_path in enumerate(images_path):
            with open(image_path, "rb") as file:
                img = file.read()
            timestamp = image_path.split('/')[-1]
            timestamp = timestamp.replace('.png', '')

            image_val = {
                "timestamp": timestamp,
                "frame_id": "LUAV",
                "data": base64.b64encode(img).decode('utf-8'),
                "format": "png"
            }

            writer.add_message(
                camera_channel_id,
                log_time=int(float(image_val["timestamp"])*1e9),
                data=json.dumps(image_val).encode("utf-8"),
                publish_time=int(float(image_val["timestamp"])*1e9))

            current_percentage = round(i/len(images_path)*100, 2)
            if current_percentage-last_print > 5:
                print(f"Imgs: {current_percentage} %")
                last_print = current_percentage

        writer.finish()


def readCSV(path):
    with open(path, "r") as f:
        values = {}
        for i, (raw_vals) in enumerate(csv.reader(f)):
            if i == 0:
                headers = [val.replace(' ', '') for val in raw_vals]
                # headers = raw_vals
                for v in headers:
                    values[v] = None
                continue
            else:
                for i, head in enumerate(headers):
                    values[head] = raw_vals[i]
            yield values


def generateChannelId(writer: Writer,
                      schema_name: str, schema_path: Path, schema_encoding: str,
                      topic: str, message_encoding: str) -> int:
    with open(schema_path, "rb") as schema_f:
        schema = schema_f.read()
    schema_id = writer.register_schema(
        name=schema_name,
        encoding=schema_encoding,
        data=schema)
    channel_id = writer.register_channel(
        topic=topic,
        message_encoding=message_encoding,
        schema_id=schema_id)
    return channel_id


def generateMcapFromCsv(path: typing.Union[str | Path],
                        schema_name: str, schema_path: Path, schema_encoding: str,
                        topic: str, message_encoding: str,
                        output_name: str) -> None:
    values = []
    for entry in readCSV(path):
        new_entry = entry.copy()
        new_entry.pop("image", None)
        new_entry["data"] = new_entry.pop("value(°c)")
        values.append(new_entry)
    with open(os.path.join(PATH_TO_OUTPUT_FOLDER, output_name+".mcap"), "wb") as f:
        writer = Writer(f)
        writer.start("x-jsonschema")
        channel_id = generateChannelId(writer, schema_name, schema_path, schema_encoding,
                                       topic, message_encoding)
        last_print = 0
        for i, (val) in enumerate(values):
            writer.add_message(
                channel_id,
                log_time=int(float(val["timestamp"])*1e9),
                data=json.dumps(val).encode("utf-8"),
                publish_time=int(float(val["timestamp"])*1e9))

            current_percentage = round(i/len(values)*100, 2)
            if current_percentage-last_print > 5:
                print(f"{current_percentage} %")
                last_print = current_percentage
        writer.finish()


def getTimeStamps():
    timestamps = []
    for timestamp, _, _, _, _, _, _, _, _, _ in data_reader(PATH_TO_CSV_FILE):
        timestamps.append(timestamp)
    return timestamps


def publishTF(path, schema_name: str, schema_path: Path, schema_encoding: str,
              topic: str, message_encoding: str,
              output_name: str):
    with open(path, 'rb') as f:
        transforms = json.load(f)
    with open(os.path.join(PATH_TO_OUTPUT_FOLDER, output_name+".mcap"), "wb") as f:
        writer = Writer(f)
        writer.start("x-jsonschema")
        channel_id = generateChannelId(
            writer, schema_name, schema_path, schema_encoding, topic, message_encoding)
        for tf in transforms:
            print(transforms[tf])
        for ts in getTimeStamps():
            writer.add_message(
                channel_id,
                log_time=int(float(ts)*1e9),
                data=json.dumps(list(transforms.values())).encode("utf-8"),
                publish_time=int(float(ts)*1e9))

        writer.finish()


def publishCameraCalib(path, schema_name: str, schema_path: Path, schema_encoding: str,
                       topic: str, message_encoding: str,
                       output_name: str):
    with open(path, 'rb') as f:
        calibs = json.load(f)
    with open(os.path.join(PATH_TO_OUTPUT_FOLDER, output_name+".mcap"), "wb") as f:
        writer = Writer(f)
        writer.start("x-jsonschema")
        channel_id = generateChannelId(
            writer, schema_name, schema_path, schema_encoding, topic, message_encoding)
        for calib in calibs:
            print(calibs[calib])
            for ts in getTimeStamps():
                writer.add_message(
                    channel_id,
                    log_time=int(float(ts)*1e9),
                    data=json.dumps(calibs[calib]).encode("utf-8"),
                    publish_time=int(float(ts)*1e9))

        writer.finish()


publishTF(os.path.join(ROOT_PATH, "code/Transforms.json"),
          "foxglove.FrameTransforms",
          os.path.join(ROOT_PATH, "code/FrameTransforms.json"),
          SchemaEncoding.JSONSchema, "tf", MessageEncoding.JSON,
          "tf")
publishCameraCalib(os.path.join(ROOT_PATH, "code/GoProCalib.json"),
                   "foxglove.CameraCalibration",
                   os.path.join(ROOT_PATH, "code/CameraCalibration.json"),
                   SchemaEncoding.JSONSchema, "go_pro_calib", MessageEncoding.JSON,
                   "go_pro_calib")
add_location_press_depth()
add_gopro()
add_gray()
add_segmentation()
add_SSS_images(False)
add_SSS_images(PATH_TO_LF_FOLDER, False, "sss_lf", "sss/lf")
generateMcapFromCsv(os.path.join(ROOT_PATH, "DATA/Temperature.csv"),
                    "foxglove.Float64Stamped",
                    os.path.join(ROOT_PATH, "code/Float64Stamped.json"),
                    SchemaEncoding.JSONSchema, "temperature", MessageEncoding.JSON,
                    "temperature")
