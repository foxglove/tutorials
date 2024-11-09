import typing
from pathlib import Path
import os
import csv
import base64
import json
import cv2

import pandas as pd
import numpy as np

from mcap.writer import Writer
from mcap.well_known import SchemaEncoding, MessageEncoding

# Define paths
ROOT_PATH = os.path.dirname(os.path.abspath(__file__))
PATH_TO_CSV_FILE = Path(
    os.path.join(ROOT_PATH, "DATA", "EstimatedState.csv"))
PATH_TO_PRESSURE_DEPTH_FILE = Path(
    os.path.join(ROOT_PATH, "DATA", "Pressure.csv"))
PATH_TO_SCHEMA_FOLDER = Path(
    os.path.join(ROOT_PATH, "schemas"))
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


def data_reader(csv_path: typing.Union[str, Path]) -> pd.DataFrame:
    """ Function to read from csv file and yield each value """
    df = pd.read_csv(csv_path)
    df.columns = df.columns.str.replace(' ', '')
    return df


def get_pressure_depth(csv_path: typing.Union[str, Path]) -> list:
    df = data_reader(csv_path)
    pressure_and_depth_values = []

    for _, row in df.iterrows():
        press_depth_val = {
            "pressure": float(row["value(hpa)"]),
            "depth": float(row["est-depth"]),
        }
        pressure_and_depth_values.append((row["timestamp"], press_depth_val))

    return pressure_and_depth_values


def get_state(csv_path: typing.Union[str, Path]) -> dict:
    df = data_reader(csv_path)
    state_topics = {}

    location_values = []
    velocity_values = []
    position_values = []
    images_path = []

    for _, row in df.iterrows():
        # Location
        loc_val = {
            "latitude": float(row["est-latitude"]),
            "longitude": float(row["est-longitude"]),
            "altitude": float(row["alt(m)"])
        }
        location_values.append((row["timestamp"], loc_val))

        # Velocity
        velocity_val = {
            "x": float(row["vx(m/s)"]),
            "y": float(row["vy(m/s)"]),
            "z": float(row["vz(m/s)"])
        }
        velocity_values.append((row["timestamp"], velocity_val))

        # Position
        position_val = {
            "x": float(row["x(m)"]),
            "y": float(row["y(m)"]),
            "z": float(row["z(m)"])
        }
        position_values.append((row["timestamp"], position_val))

        # GoPro
        images_path.append((row["timestamp"], os.path.join(
            PATH_TO_IMAGES_FOLDER, os.path.basename(row["image"]))))

    state_topics["location"] = location_values
    state_topics["velocity"] = velocity_values
    state_topics["position"] = position_values
    state_topics["cam0"] = images_path

    return state_topics


def generate_channel_id(channels: dict, writer: Writer, json_name: str, topic: str):
    with open(os.path.join(PATH_TO_SCHEMA_FOLDER, json_name+".json"), "rb") as f:
        schema = f.read()
        pressure_schema_id = writer.register_schema(
            name="foxglove."+json_name,
            encoding=SchemaEncoding.JSONSchema,
            data=schema)
        pressure_channel_id = writer.register_channel(
            topic=topic,
            message_encoding=MessageEncoding.JSON,
            schema_id=pressure_schema_id)
        channels[topic] = pressure_channel_id


def write_mcap(topics: dict):
    # Open mcap file to write
    with open(os.path.join(PATH_TO_OUTPUT_FOLDER, "subpipe.mcap"), "wb") as f:
        writer = Writer(f)
        writer.start("x-jsonschema")

        # Generate channels for each topic
        channels = {}
        generate_channel_id(
            channels, writer, "PressureDepth", "pressure_depth")
        generate_channel_id(
            channels, writer, "LocationFix", "location")
        generate_channel_id(
            channels, writer, "Vector3", "velocity")
        generate_channel_id(
            channels, writer, "Vector3", "position")
        generate_channel_id(
            channels, writer, "CompressedImage", "cam0")

        # Write mcap
        for i, (topic, values) in enumerate(topics.items()):
            print(f"Topic {topic}: {i+1}/{len(topics)}")
            last_print = 0
            for i, val in enumerate(values):
                if topic in ["cam0", "cam1"]:
                    with open(val[1], "rb") as file:
                        img = file.read()
                        image_val = {
                            "timestamp": val[0],
                            "frame_id": "gopro_link",
                            "data": base64.b64encode(img).decode('utf-8'),
                            "format": "jpg"
                        }
                    writer.add_message(
                        channels[topic],
                        log_time=int(float(image_val["timestamp"])*1e9),
                        data=json.dumps(image_val).encode("utf-8"),
                        publish_time=int(float(image_val["timestamp"])*1e9))
                else:
                    writer.add_message(
                        channels[topic],
                        log_time=int(float(val[0])*1e9),
                        data=json.dumps(val[1]).encode("utf-8"),
                        publish_time=int(float(val[0])*1e9))

                current_percentage = round(i/len(values)*100, 2)
                if current_percentage-last_print > 5:
                    print(f"{current_percentage}%")
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


def getTimestamps(csv_path: typing.Union[str, Path]):
    df = data_reader(csv_path)
    timestamps = []
    for data in df["timestamp"]:
        timestamps.append(data)
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


topics = {}

# topics["timestamps"] = getTimestamps(PATH_TO_CSV_FILE)
topics["pressure_depth"] = get_pressure_depth(PATH_TO_PRESSURE_DEPTH_FILE)
topics.update(get_state(PATH_TO_CSV_FILE))

write_mcap(topics)


# publishTF(os.path.join(ROOT_PATH, "code/Transforms.json"),
#           "foxglove.FrameTransforms",
#           os.path.join(ROOT_PATH, "code/FrameTransforms.json"),
#           SchemaEncoding.JSONSchema, "tf", MessageEncoding.JSON,
#           "tf")
# publishCameraCalib(os.path.join(ROOT_PATH, "code/GoProCalib.json"),
#                    "foxglove.CameraCalibration",
#                    os.path.join(ROOT_PATH, "code/CameraCalibration.json"),
#                    SchemaEncoding.JSONSchema, "go_pro_calib", MessageEncoding.JSON,
#                    "go_pro_calib")
# add_location_press_depth()
# add_gopro()
# add_gray()
# add_segmentation()
# add_SSS_images(False)
# add_SSS_images(PATH_TO_LF_FOLDER, False, "sss_lf", "sss/lf")
# generateMcapFromCsv(os.path.join(ROOT_PATH, "DATA/Temperature.csv"),
#                     "foxglove.Float64Stamped",
#                     os.path.join(ROOT_PATH, "code/Float64Stamped.json"),
#                     SchemaEncoding.JSONSchema, "temperature", MessageEncoding.JSON,
#                     "temperature")
