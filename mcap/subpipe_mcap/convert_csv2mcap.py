import typing
from pathlib import Path
import os
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
PATH_TO_TEMP_FILE = Path(
    os.path.join(ROOT_PATH, "DATA", "Temperature.csv"))
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
# Auxiliary paths for PNG images needed for Sonar
PATH_TO_HF_PNG_FOLDER = Path(
    os.path.join(ROOT_PATH, "DATA", "SSS_HF_images_png"))
PATH_TO_LF_PNG_FOLDER = Path(
    os.path.join(ROOT_PATH, "DATA", "SSS_LF_images_png"))
PATH_TO_OUTPUT_FOLDER = Path(
    os.path.join(ROOT_PATH, "output"))
PATH_TO_VIDEO_FOLDER = Path(
    os.path.join(ROOT_PATH, "video"))

# Create output folders
if not os.path.exists(PATH_TO_OUTPUT_FOLDER):
    os.mkdir(PATH_TO_OUTPUT_FOLDER)

if not os.path.exists(PATH_TO_VIDEO_FOLDER):
    os.mkdir(PATH_TO_VIDEO_FOLDER)

if not os.path.exists(PATH_TO_HF_PNG_FOLDER):
    os.mkdir(PATH_TO_HF_PNG_FOLDER)

if not os.path.exists(PATH_TO_LF_PNG_FOLDER):
    os.mkdir(PATH_TO_LF_PNG_FOLDER)


def data_reader(csv_path: typing.Union[str, Path]) -> pd.DataFrame:
    """ Function to read from csv file and return a pandas DF  """
    df = pd.read_csv(csv_path)
    df.columns = df.columns.str.replace(' ', '')
    return df


def get_pressure_depth(csv_path: typing.Union[str, Path]) -> list:
    """ Reads the pressure and depth csv file """
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
    """ Reads the rpm csv file """
    df = data_reader(csv_path)
    rpm_values = []

    for _, row in df.iterrows():
        rpm_val = {
            "data": float(row["value(rpm)"]),
        }
        rpm_values.append((row["timestamp"], rpm_val))

    return rpm_values


def get_temperature(csv_path: typing.Union[str, Path]) -> list:
    """ Reads the temperature csv file """
    df = data_reader(csv_path)
    temp_values = []

    for _, row in df.iterrows():
        temp_val = {
            "timestamp": float(row["timestamp"]),
            "data": float(row["value(°c)"]),
        }
        temp_values.append((row["timestamp"], temp_val))

    return temp_values


def get_state(csv_path: typing.Union[str, Path]) -> dict:
    """ Reads the estimated state csv file and generates topics for values and images """
    df = data_reader(csv_path)
    state_topics = {}

    location_values = []
    velocity_values = []
    position_values = []
    tf_values = []
    sea_values = []
    cam0_paths = []
    cam0_info = []
    cam1_paths = []
    cam1_info = []
    sss_hf_paths = []
    sss_lf_paths = []
    img_segmentation_paths = []
    img_label_paths = []

    # GoPro calibration file
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

        # Velocity values
        velocity_val = {
            "x": float(row["vx(m/s)"]),
            "y": float(row["vy(m/s)"]),
            "z": float(row["vz(m/s)"])
        }
        velocity_values.append((row["timestamp"], velocity_val))

        # Position as frame transform
        position_val = {
            "parent_frame_id": "initial",
            "child_frame_id": "LUAV",
            "translation": {
                "x": float(row["x(m)"]-df["x(m)"][0]),
                "y": float(row["y(m)"]-df["y(m)"][0]),
                "z": float(row["alt(m)"]-df["alt(m)"][0])
            },
            "rotation": {
                "x": 0.7071068,
                "y": -0.7071068,
                "z": 0,
                "w": 0
            }
        }
        position_values.append((row["timestamp"], position_val))

        # GoPro TF to LUAV (data from config.yaml)
        tf_val = {
            "parent_frame_id": "LUAV",
            "child_frame_id": "gopro",
            "translation": {
                "x": 0.619,
                "y": 0,
                "z": -0.120
            },
            "rotation": {
                "x": 0.7071068,
                "y": -0.7071068,
                "z": 0,
                "w": 0
            }
        }
        tf_values.append((row["timestamp"], tf_val))

        # Gray TF to LUAV (data from config.yaml)
        tf_val = {
            "parent_frame_id": "LUAV",
            "child_frame_id": "gray",
            "translation": {
                "x": -0.813,
                "y": 0,
                "z": -0.045
            },
            "rotation": {
                "x": 0.7071068,
                "y": -0.7071068,
                "z": 0,
                "w": 0
            }
        }
        tf_values.append((row["timestamp"], tf_val))

        # Sealevel TF to initial
        tf_val = {
            "parent_frame_id": "initial",
            "child_frame_id": "sea_level",
            "translation": {
                "x": 0,
                "y": 0,
                "z": -df["depth(m)"][0]
            },
            "rotation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "w": 1
            }
        }
        tf_values.append((row["timestamp"], tf_val))

        # Sea level "scene" representation
        sea_val = {
            "entities": [
                {
                    "frame_id": "sea_level",
                    "id": "sea0",
                    "cubes": [
                        {
                            "pose": {
                                "position": {
                                    "x": 0,
                                    "y": 0,
                                    "z": 0
                                },
                                "orientation": {
                                    "x": 0,
                                    "y": 0,
                                    "z": 0,
                                    "w": 1
                                }
                            },
                            "size": {
                                "x": 200,
                                "y": 200,
                                "z": 0.25
                            },
                            "color": {
                                "r": 0,
                                "g": 0,
                                "b": 1,
                                "a": 0.75
                            }
                        }
                    ]
                }
            ]
        }
        sea_values.append((row["timestamp"], sea_val))

        # GoPro images with calibration info
        cam0_paths.append((row["timestamp"], os.path.join(
            PATH_TO_IMAGES_FOLDER, os.path.basename(row["image"]))))
        calib_val = {
            "timestamp": row["timestamp"],
            "frame_id": "gopro",
            "width": calibration["width"],
            "height": calibration["height"],
            "distortion_model": calibration["distortion_model"],
            "D": calibration["D"],
            "K": calibration["K"],
            "R": calibration["R"],
            "P": calibration["P"],
        }
        cam0_info.append((row["timestamp"], calib_val))

    # Gray images with calibration info
    for file in os.listdir(PATH_TO_GRAY_FOLDER):
        timestamp = float(file.replace(".jpg", ""))
        cam1_paths.append((timestamp, os.path.join(PATH_TO_GRAY_FOLDER, file)))
        calib_val = {
            "timestamp": timestamp,
            "frame_id": "gray",
            "width": 1936,
            "height": 1216,
            "distortion_model": calibration["distortion_model"],
            "D": calibration["D"],
            "K": calibration["K"],
            "R": calibration["R"],
            "P": calibration["P"],
        }
        cam1_info.append((timestamp, calib_val))

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

    # Add topics values
    state_topics["location"] = location_values
    state_topics["velocity"] = velocity_values
    state_topics["position"] = position_values
    state_topics["tf"] = tf_values
    state_topics["sea"] = sea_values
    state_topics["cam0"] = cam0_paths
    state_topics["cam0/camera_info"] = cam0_info
    state_topics["cam1"] = cam1_paths
    state_topics["cam1/camera_info"] = cam1_info
    state_topics["sss/hf"] = sss_hf_paths
    state_topics["sss/lf"] = sss_lf_paths
    state_topics["cam0/segmentation"] = img_segmentation_paths
    state_topics["cam0/label"] = img_label_paths

    return state_topics


def generate_channel_id(channels: dict, writer: Writer, json_name: str, topic: str):
    """ Generate a topic channel_id for the specified message type """
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


def write_mcap(topics_to_write: dict, topics_channels: list):
    """ Write the MCAP file """
    # Open mcap file to write
    with open(os.path.join(PATH_TO_OUTPUT_FOLDER, "subpipe.mcap"), "wb") as f:
        writer = Writer(f)
        writer.start("x-jsonschema")

        # Generate channels for each topic
        channels = {}
        for topic_channel in topics_channels:
            generate_channel_id(
                channels, writer, topic_channel[0], topic_channel[1])

        # Cam topic to frame dict
        cam_frame = {
            "cam0": "gopro",
            "cam1": "gray",
            "sss/hf": "sss",
            "cam0/segmentation": "gopro",
            "cam0/label": "gopro"
        }
        # Write mcap
        for i, (topic, values) in enumerate(topics_to_write.items()):
            print(f"Topic {topic}: {i+1}/{len(topics_to_write)}")
            last_print = 0
            for i, val in enumerate(values):
                # Camera topics are different
                if topic in ["cam0", "cam1", "sss/hf", "sss/lf", "cam0/segmentation", "cam0/label"]:
                    if ("sss" in topic) or ("cam0/" in topic):
                        img_format = "png"
                    else:
                        img_format = "jpg"
                    with open(val[1], "rb") as file:
                        img = file.read()
                        image_val = {
                            "timestamp": val[0],
                            "frame_id": cam_frame[topic],
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


def main():
    topics = {}

    # Get topic values
    topics["pressure_depth"] = get_pressure_depth(PATH_TO_PRESSURE_DEPTH_FILE)
    topics["rpm"] = get_rpm(PATH_TO_RPM_FILE)
    topics["temperature"] = get_temperature(PATH_TO_TEMP_FILE)
    topics.update(get_state(PATH_TO_CSV_FILE))

    # Define channels as (schema_name, topic_name)
    channels = [("PressureDepth", "pressure_depth"),
                ("Float64Stamped", "rpm"),
                ("Float64Stamped", "temperature"),
                ("LocationFix", "location"),
                ("Vector3", "velocity"),
                ("FrameTransform", "position"),
                ("FrameTransform", "tf"),
                ("SceneUpdate", "sea"),
                ("CompressedImage", "cam0"),
                ("CompressedImage", "cam1"),
                ("CompressedImage", "sss/hf"),
                ("CompressedImage", "sss/lf"),
                ("CompressedImage", "cam0/segmentation"),
                ("CompressedImage", "cam0/label"),
                ("CameraCalibration", "cam0/camera_info")
                ]

    write_mcap(topics, channels)


if __name__ == "__main__":
    main()
