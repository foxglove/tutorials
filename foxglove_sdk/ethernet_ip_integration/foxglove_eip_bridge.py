
import datetime
import json
import logging
import time
from typing import Dict, Any

import foxglove
from foxglove import Channel, Schema
from foxglove.websocket import (
    Capability,
    ChannelView,
    Client,
    ServerListener,
)
from pylogix import PLC


# Define schema for timeseries data
timeseries_schema = {
    "type": "object",
    "properties": {
        "timestamp": {"type": "number"},
        "value": {"type": "number"},
        "tag_name": {"type": "string"},
    },
}

boolean_schema = {
    "type": "object",
    "properties": {
        "timestamp": {"type": "number"},
        "value": {"type": "boolean"},
        "tag_name": {"type": "string"},
    },
}

# All tags from the EtherNet/IP server configuration
ALL_TAGS = [
    # Process variables
    "Temperature",
    "Pressure",
    "Flow_Rate",
    "Level",
    "Vibration",

    # Control variables
    "Motor_Running",
    "Pump_Speed",
    "Valve_Position",
    "Pump_Enable",
    "Emergency_Stop",

    # Status and alarms
    "High_Temp_Alarm",
    "Low_Pressure_Alarm",
    "System_Status",
    "Error_Code",
    "Alarm_Count",

    # Production data
    "Production_Count",
    "Runtime_Hours",
    "Batch_Number",
    "Recipe_Number",
    "Quality_Rating",

    # Setpoints
    "Setpoint_Temp",
    "Setpoint_Pressure",
    "Setpoint_Flow",

    # Tank levels
    "Tank_Level_1",
    "Tank_Level_2",
    "Tank_Level_3",
    "Tank_Level_4",

    # Sensor arrays
    "Sensor_1",
    "Sensor_2",
    "Sensor_3",
    "Sensor_4",

    # Motor data
    "Motor_Current",
    "Motor_Voltage",
    "Motor_Power",
    "Motor_Temp",
]

# Tags that are boolean values
BOOLEAN_TAGS = [
    "Motor_Running",
    "Pump_Enable",
    "Emergency_Stop",
    "High_Temp_Alarm",
    "Low_Pressure_Alarm",
]


class EIPServerListener(ServerListener):
    def __init__(self) -> None:
        self.subscribers: dict[int, set[str]] = {}

    def has_subscribers(self) -> bool:
        return len(self.subscribers) > 0

    def on_subscribe(self, client: Client, channel: ChannelView) -> None:
        logging.info(f"Client {client} subscribed to channel {channel.topic}")
        self.subscribers.setdefault(client.id, set()).add(channel.topic)

    def on_unsubscribe(self, client: Client, channel: ChannelView) -> None:
        logging.info(f"Client {client} unsubscribed from channel {channel.topic}")
        if client.id in self.subscribers:
            self.subscribers[client.id].discard(channel.topic)
            if not self.subscribers[client.id]:
                del self.subscribers[client.id]


def create_channels() -> Dict[str, Channel]:
    """Create Foxglove channels for all tags"""
    channels = {}

    for tag in ALL_TAGS:
        # Use boolean schema for boolean tags, numeric for others
        schema = boolean_schema if tag in BOOLEAN_TAGS else timeseries_schema

        # Create channel with topic name based on tag
        topic = f"/eip/{tag.lower().replace(' ', '_').replace('@', '_at_').replace('/', '_')}"

        channels[tag] = Channel(
            topic=topic,
            message_encoding="json",
            schema=Schema(
                name=f"{tag}_data",
                encoding="jsonschema",
                data=json.dumps(schema).encode("utf-8"),
            ),
        )

    return channels


def main() -> None:
    foxglove.set_log_level(logging.INFO)

    listener = EIPServerListener()

    # Start Foxglove server
    server = foxglove.start_server(
        server_listener=listener,
        capabilities=[Capability.ClientPublish],
        supported_encodings=["json"],
    )

    # Create channels for all tags
    channels = create_channels()

    # Initialize PLC connection
    plc_host = '0.0.0.0'  # EtherNet/IP server address
    plc_port = 44818      # Default EtherNet/IP port

    logging.info(f"Starting EtherNet/IP to Foxglove bridge")
    logging.info(f"Connecting to PLC at {plc_host}:{plc_port}")
    logging.info(f"Streaming {len(ALL_TAGS)} tags to Foxglove")

    try:
        with PLC() as comm:
            comm.IPAddress = plc_host
            comm.Micro800 = True

            counter = 0
            while True:
                now = time.time()

                # Read and stream all tags
                for tag_name in ALL_TAGS:
                    try:
                        # Read value from PLC
                        result = comm.Read(tag_name)

                        if result.Status == "Success" and result.Value is not None:
                            # Prepare message based on tag type
                            if tag_name in BOOLEAN_TAGS:
                                message = {
                                    "timestamp": now,
                                    "value": bool(result.Value),
                                    "tag_name": tag_name,
                                }
                            else:
                                # Convert to float for numeric values
                                try:
                                    value = float(result.Value)
                                except (ValueError, TypeError):
                                    value = 0.0

                                message = {
                                    "timestamp": now,
                                    "value": value,
                                    "tag_name": tag_name,
                                }

                            # Log to channel
                            if tag_name in channels:
                                channels[tag_name].log(json.dumps(message).encode("utf-8"))

                        else:
                            logging.warning(f"Failed to read {tag_name}: {result.Status}")

                    except Exception as e:
                        logging.error(f"Error reading tag {tag_name}: {e}")

                counter += 1

                # Log status every 100 iterations
                if counter % 100 == 0:
                    logging.info(f"Streamed {counter * len(ALL_TAGS)} data points to Foxglove")

                time.sleep(0.1)  # 10Hz update rate

                # Wait if no subscribers
                while not listener.has_subscribers():
                    logging.info("Waiting for Foxglove clients to connect...")
                    time.sleep(2)

    except KeyboardInterrupt:
        logging.info("Shutting down bridge...")
    except Exception as e:
        logging.error(f"Bridge error: {e}")
    finally:
        server.stop()
        logging.info("Bridge stopped")


if __name__ == "__main__":
    main()
