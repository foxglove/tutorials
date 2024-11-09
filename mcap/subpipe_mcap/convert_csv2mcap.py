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
PATH_TO_RPM_FILE = Path(
    os.path.join(ROOT_PATH, "DATA", "Rpm.csv"))
PATH_TO_SCHEMA_FOLDER = Path(
    os.path.join(ROOT_PATH, "schemas"))
PATH_TO_CALIB_FILE = Path(
    os.path.join(PATH_TO_SCHEMA_FOLDER, "GoProCalib.json"))
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
PATH_TO_HF_PNG_FOLDER = Path(
    os.path.join(ROOT_PATH, "DATA", "SSS_HF_images_png"))
PATH_TO_LF_PNG_FOLDER = Path(
    os.path.join(ROOT_PATH, "DATA", "SSS_LF_images_png"))
PATH_TO_OUTPUT_FOLDER = Path(
    os.path.join(ROOT_PATH, "output"))
PATH_TO_VIDEO_FOLDER = Path(
    os.path.join(ROOT_PATH, "video"))

# Create output paths
if not os.path.exists(PATH_TO_OUTPUT_FOLDER):
    os.mkdir(PATH_TO_OUTPUT_FOLDER)

if not os.path.exists(PATH_TO_VIDEO_FOLDER):
    os.mkdir(PATH_TO_VIDEO_FOLDER)

if not os.path.exists(PATH_TO_HF_PNG_FOLDER):
    os.mkdir(PATH_TO_HF_PNG_FOLDER)

if not os.path.exists(PATH_TO_LF_PNG_FOLDER):
    os.mkdir(PATH_TO_LF_PNG_FOLDER)


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


def get_rpm(csv_path: typing.Union[str, Path]) -> list:
    df = data_reader(csv_path)
    rpm_values = []

    for _, row in df.iterrows():
        rpm_val = {
            "data": float(row["value(rpm)"]),
        }
        rpm_values.append((row["timestamp"], rpm_val))

    return rpm_values


def get_state(csv_path: typing.Union[str, Path]) -> dict:
    df = data_reader(csv_path)
    state_topics = {}

    location_values = []
    velocity_values = []
    position_values = []
    cam0_paths = []
    cam0_info = []
    cam1_paths = []
    sss_hf_paths = []
    sss_lf_paths = []
    img_segmentation_paths = []
    img_label_paths = []

    with open(PATH_TO_CALIB_FILE, 'rb') as f:
        calibration = json.load(f)

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
            "parent_frame_id": "initial",
            "child_frame_id": "LUAV",
            "translation": {
                "x": float(row["x(m)"]-df["x(m)"][0]),
                "y": float(row["y(m)"]-df["y(m)"][0]),
                "z": float(row["alt(m)"]-df["alt(m)"][0])
            },
            "rotation": {
                "x": 0,
                "y": 0,
                "z": -0.7071068,
                "w": 0.7071068
            }
        }
        position_values.append((row["timestamp"], position_val))

        # GoPro
        cam0_paths.append((row["timestamp"], os.path.join(
            PATH_TO_IMAGES_FOLDER, os.path.basename(row["image"]))))
        cam0_info.append((row["timestamp"], calibration))

    # Gray images
    for file in os.listdir(PATH_TO_GRAY_FOLDER):
        timestamp = file.replace(".jpg", "")
        cam1_paths.append((timestamp, os.path.join(PATH_TO_GRAY_FOLDER, file)))

    # SSS HF images
    for i, file in enumerate(os.listdir(PATH_TO_HF_FOLDER)):
        timestamp = file.replace(".pbm", "")
        # Convert to PNG
        png_filename = file.replace('.pbm', '.png')
        if not os.path.exists(os.path.join(PATH_TO_HF_PNG_FOLDER, png_filename)):
            print(
                f"Converting HF image '.pbm' to '.png': {i}/{len(os.listdir(PATH_TO_HF_FOLDER))}")
            img = cv2.imread(os.path.join(
                os.path.join(PATH_TO_HF_FOLDER, file)))
            cv2.imwrite(os.path.join(
                PATH_TO_HF_PNG_FOLDER, png_filename), img)
        sss_hf_paths.append(
            (timestamp, os.path.join(PATH_TO_HF_PNG_FOLDER, png_filename)))

    # SSS LF images
    for i, file in enumerate(os.listdir(PATH_TO_LF_FOLDER)):
        timestamp = file.replace(".pbm", "")
        # Convert to PNG
        png_filename = file.replace('.pbm', '.png')
        if not os.path.exists(os.path.join(PATH_TO_LF_PNG_FOLDER, png_filename)):
            print(
                f"Converting LF image '.pbm' to '.png': {i}/{len(os.listdir(PATH_TO_HF_FOLDER))}")
            img = cv2.imread(os.path.join(
                os.path.join(PATH_TO_LF_FOLDER, file)))
            cv2.imwrite(os.path.join(
                PATH_TO_LF_PNG_FOLDER, png_filename), img)
        sss_lf_paths.append(
            (timestamp, os.path.join(PATH_TO_LF_PNG_FOLDER, png_filename)))

    # Segmentation imgs
    for file in os.listdir(PATH_TO_SEGMENT_FOLDER):
        if "_label" in file:
            continue
        timestamp = file.replace(".png", "")
        label_filename = file.replace(".png", "")+"_label.png"
        img_segmentation_paths.append(
            (timestamp, os.path.join(PATH_TO_SEGMENT_FOLDER, file)))
        img_label_paths.append(
            (timestamp, os.path.join(PATH_TO_SEGMENT_FOLDER, label_filename)))

    state_topics["location"] = location_values
    state_topics["velocity"] = velocity_values
    state_topics["tf"] = position_values
    state_topics["cam0"] = cam0_paths
    state_topics["cam0/camera_info"] = cam0_info
    state_topics["cam1"] = cam1_paths
    state_topics["sss/hf"] = sss_hf_paths
    state_topics["sss/lf"] = sss_lf_paths
    state_topics["cam0/segmentation"] = img_segmentation_paths
    state_topics["cam0/label"] = img_label_paths

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
            channels, writer, "Float64Stamped", "rpm")
        generate_channel_id(
            channels, writer, "LocationFix", "location")
        generate_channel_id(
            channels, writer, "Vector3", "velocity")
        generate_channel_id(
            channels, writer, "FrameTransform", "tf")
        generate_channel_id(
            channels, writer, "CompressedImage", "cam0")
        generate_channel_id(
            channels, writer, "CompressedImage", "cam1")
        generate_channel_id(
            channels, writer, "CompressedImage", "sss/hf")
        generate_channel_id(
            channels, writer, "CompressedImage", "sss/lf")
        generate_channel_id(
            channels, writer, "CompressedImage", "cam0/segmentation")
        generate_channel_id(
            channels, writer, "CompressedImage", "cam0/label")
        generate_channel_id(
            channels, writer, "CameraCalibration", "cam0/camera_info")

        # Write mcap
        for i, (topic, values) in enumerate(topics.items()):
            print(f"Topic {topic}: {i+1}/{len(topics)}")
            last_print = 0
            for i, val in enumerate(values):
                if topic in ["cam0", "cam1", "sss/hf", "sss/lf", "cam0/segmentation", "cam0/label"]:
                    if ("sss" in topic) or ("cam0/" in topic):
                        img_format = "png"
                    else:
                        img_format = "jpg"
                    with open(val[1], "rb") as file:
                        img = file.read()
                        image_val = {
                            "timestamp": val[0],
                            "frame_id": "LUAV",
                            "data": base64.b64encode(img).decode('utf-8'),
                            "format": img_format
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


topics = {}

topics["pressure_depth"] = get_pressure_depth(PATH_TO_PRESSURE_DEPTH_FILE)
topics["rpm"] = get_rpm(PATH_TO_RPM_FILE)
topics.update(get_state(PATH_TO_CSV_FILE))

write_mcap(topics)
