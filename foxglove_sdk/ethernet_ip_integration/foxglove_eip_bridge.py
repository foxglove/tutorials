
import datetime
import json
import logging
import time
from typing import Dict

import foxglove
from foxglove import Channel, Schema
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

def create_channels() -> Dict[str, Channel]:
    """Create Foxglove channels for all tags"""
    channels = {}

    for tag in ALL_TAGS:
        # Create channel with topic name based on tag
        topic = f"/eip/{tag.lower().replace(' ', '_')}"

        channels[tag] = Channel(
            topic=topic,
            message_encoding="json",
            schema=Schema(
                name=f"{tag}_data",
                encoding="jsonschema",
                data=json.dumps(timeseries_schema).encode("utf-8"),
            ),
        )
    return channels


def main() -> None:
    foxglove.set_log_level(logging.INFO)

    server = foxglove.start_server()
    now_str = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    file_name = f"foxglove_eip_bridge_{now_str}.mcap"
    writer = foxglove.open_mcap(file_name) # Comment out if not logging to mcap

    # Create channels for all tags
    channels = create_channels()

    # Initialize PLC connection
    plc_host = '0.0.0.0'  # EtherNet/IP server address

    logging.info(f"Starting EtherNet/IP to Foxglove bridge")
    logging.info(f"Connecting to PLC at {plc_host}")
    logging.info(f"Streaming {len(ALL_TAGS)} tags to Foxglove")

    try:
        with PLC() as comm:
            comm.IPAddress = plc_host
            comm.Micro800 = True

            while True:
                now = time.time()

                # Read and stream all tags
                for tag_name in ALL_TAGS:
                    try:
                        # Read value from PLC
                        result = comm.Read(tag_name)

                        if result.Status == "Success" and result.Value is not None:
                            message = {
                                "timestamp": now,
                                "value": result.Value,
                                "tag_name": tag_name,
                            }

                            # Log to channel
                            if tag_name in channels:
                                channels[tag_name].log(json.dumps(message).encode("utf-8"))

                        else:
                            logging.warning(f"Failed to read {tag_name}: {result.Status}")

                    except Exception as e:
                        logging.error(f"Error reading tag {tag_name}: {e}")

                time.sleep(0.1)
    except KeyboardInterrupt:
        logging.info("Shutting down bridge...")
    except Exception as e:
        logging.error(f"Bridge error: {e}")
    finally:
        server.stop()
        writer.close()
        logging.info("Bridge stopped")

if __name__ == "__main__":
    main()
