import csv
import typing
from pathlib import Path
import os
import json

import numpy as np
import pandas as pd

from mcap.writer import Writer
from mcap.well_known import SchemaEncoding, MessageEncoding

CSV_FILE = "mcap_example.csv"
CSV_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), CSV_FILE)
OUTPUT_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)))
SCHEMAS_PATH = os.path.join(os.path.dirname(
    os.path.abspath(__file__)), "schemas")


# Open
df = pd.read_csv(CSV_PATH)
df["timestamp"] *= 1e9
df["timestamp"] = df["timestamp"].astype("uint")
print(df["timestamp"])


def write_mcap(filename, data_list):
    with open(os.path.join(OUTPUT_PATH, filename+".mcap"), "wb") as f:
        writer = Writer(f)
        writer.start("x-jsonschema")

        for topic in data_list:
            topic_name = topic["name"]
            topic_data = topic["data"]
            schema_name = topic["schema"]

            json_filename = os.path.join(SCHEMAS_PATH, schema_name+".json")

            # Prepare schema_id and channel_id
            with open(Path(__file__).parent / json_filename, "rb") as f:
                schema = f.read()
                schema_id = writer.register_schema(
                    name="foxglove."+schema_name,
                    encoding=SchemaEncoding.JSONSchema,
                    data=schema)
                channel_id = writer.register_channel(
                    topic=topic_name,
                    message_encoding=MessageEncoding.JSON,
                    schema_id=schema_id)
            # To show progress
            last_print = 0

            for i, (data) in enumerate(topic_data["values"]):
                writer.add_message(
                    channel_id,
                    log_time=topic_data["timestamp"][i],
                    data=json.dumps(data).encode("utf-8"),
                    publish_time=topic_data["timestamp"][i])

                current_percentage = round(i/len(topic_data["values"])*100, 2)
                if current_percentage-last_print > 5:
                    print(f"{current_percentage} %")
                    last_print = current_percentage

        writer.finish()


data_combined = []
# Prepare battery data
battery_data = {
    "timestamp": df["timestamp"],
    "values": [],
}
for bat in df["battery"]:
    new_value = {
        "percentage": bat,
    }
    battery_data["values"].append(new_value)

battery = {
    "name": "battery",
    "data": battery_data,
    "schema": "Battery"
}
data_combined.append(battery)

# Prepare speed data
speed_data = {
    "timestamp": df["timestamp"],
    "values": [],
}
for x, y, z in zip(df["speed_x"], df["speed_y"], df["speed_z"]):
    new_value = {
        "x": x,
        "y": y,
        "z": z,
    }
    speed_data["values"].append(new_value)
speed = {
    "name": "speed",
    "data": speed_data,
    "schema": "Vector3"
}
data_combined.append(speed)


# Prepare location data
location_data = {
    "timestamp": df["timestamp"],
    "values": [],
}
for lat, lon in zip(df["latitude"], df["longitude"]):
    new_value = {
        "latitude": lat,
        "longitude": lon,
    }
    location_data["values"].append(new_value)
location = {
    "name": "location",
    "data": location_data,
    "schema": "LocationFix"
}
data_combined.append(location)

write_mcap("tutorial", data_combined)
