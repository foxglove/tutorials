import foxglove
import time
import logging
import numpy as np
import math
import json
import queue
#import datetime

from lerobot.common.robots.so100_follower import SO100FollowerConfig, SO100Follower

from foxglove.schemas import (
    RawImage,
    FrameTransforms,
    FrameTransform,
    Vector3,
    Quaternion,
)
from foxglove.channels import RawImageChannel
from foxglove.websocket import (
    Capability,
    ChannelView,
    Client,
    ClientChannel,
    ServerListener,
)

from lerobot.common.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.common.cameras.opencv.camera_opencv import OpenCVCamera
from lerobot.common.cameras.configs import ColorMode, Cv2Rotation

from urdfpy import URDF
# Hack to ensure np.float works with ancient urdfpy version
if not hasattr(np, 'float'):
    np.float = float

WORLD_FRAME_ID = "world"
BASE_FRAME_ID = "base"
RATE_HZ = 30.0
URDF_FILE = "SO100/so100.urdf"
ROBOT_NAME = "foxglove_so100"
WRIST_CAM_ID = 4

CMD_RECORD = "record"
CMD_STOP = "stop"
CMD_REPLAY = "replay"

class TopicListener(ServerListener):
    def __init__(self) -> None:
        # Map client id -> set of subscribed topics
        self.subscribers: dict[int, set[str]] = {}
        self.command_queue = queue.Queue()
        # Recording state
        self.is_recording = False
        self.recorded_data = []
        # Replay state
        self.is_replaying = False
        self.replay_index = 0

    def has_subscribers(self) -> bool:
        return len(self.subscribers) > 0

    def on_subscribe(
        self,
        client: Client,
        channel: ChannelView,
    ) -> None:
        """
        Called by the server when a client subscribes to a channel.
        We'll use this and on_unsubscribe to simply track if we have any subscribers at all.
        """
        logging.info(f"Client {client} subscribed to channel {channel.topic}")
        self.subscribers.setdefault(client.id, set()).add(channel.topic)

    def on_unsubscribe(
        self,
        client: Client,
        channel: ChannelView,
    ) -> None:
        """
        Called by the server when a client unsubscribes from a channel.
        """
        logging.info(f"Client {client} unsubscribed from channel {channel.topic}")
        self.subscribers[client.id].remove(channel.topic)
        if not self.subscribers[client.id]:
            del self.subscribers[client.id]

    def on_client_advertise(
        self,
        client: Client,
        channel: ClientChannel,
    ) -> None:
        """
        Called when a client advertises a new channel.
        """
        logging.info(f"Client {client.id} advertised channel: {channel.id}")
        logging.info(f"  Topic: {channel.topic}")
        logging.info(f"  Encoding: {channel.encoding}")
        logging.info(f"  Schema name: {channel.schema_name}")
        logging.info(f"  Schema encoding: {channel.schema_encoding}")
        logging.info(f"  Schema: {channel.schema!r}")

    def on_message_data(
        self,
        client: Client,
        client_channel_id: int,
        data: bytes,
    ) -> None:
        """
        This handler demonstrates receiving messages from the client.
        You can send messages from Foxglove app in the publish panel:
        https://docs.foxglove.dev/docs/visualization/panels/publish
        """
        logging.info(f"Message from client {client.id} on channel {client_channel_id}")
        logging.info(f"Data: {data!r}")

        # Try to parse as JSON command - we'll assume it's from /lerobot_action if it has a command field
        try:
            command_data = json.loads(data.decode('utf-8'))
            if 'command' in command_data:
                command = command_data['command']
                logging.info(f"Received command: {command}")
                self.command_queue.put(command)
        except (json.JSONDecodeError, UnicodeDecodeError) as e:
            # Not a JSON command, just log it as debug
            logging.debug(f"Could not parse message as JSON command: {e}")

    def on_client_unadvertise(
        self,
        client: Client,
        client_channel_id: int,
    ) -> None:
        """
        Called when a client unadvertises a new channel.
        """
        logging.info(f"Client {client.id} unadvertised channel: {client_channel_id}")

def rot_matrix_to_quat(R):
    """
    Convert a 3x3 rotation matrix to quaternion [x, y, z, w].
    """
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    else:
        if (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
    return np.array([x, y, z, w], dtype=np.float64)

def main():
    foxglove.set_log_level(logging.INFO)

    print(f"Loading URDF from {URDF_FILE} ...")
    robot = URDF.load(URDF_FILE)

    # (Optional) Log data to MCAP files
    #now_str = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    #file_name = f"so_arm_100_{now_str}.mcap"
    #writer = foxglove.open_mcap(file_name)

    # Start the Foxglove server

    listener = TopicListener()

    server = foxglove.start_server(
        server_listener=listener,
        capabilities=[Capability.ClientPublish],
        supported_encodings=["json"],
    )


    # Setup camera for logging
    cam_config = OpenCVCameraConfig(
        index_or_path=WRIST_CAM_ID,
        fps=30,
        width=640,
        height=480,
        color_mode=ColorMode.RGB,
        rotation=Cv2Rotation.NO_ROTATION
    )
    camera = OpenCVCamera(cam_config)
    camera.connect()
    image_channel = RawImageChannel(topic="wrist_image")

    print("Open Foxglove Studio and connect to ws://localhost:8765")

    config = SO100FollowerConfig(port="/dev/ttyUSB0", id=ROBOT_NAME, use_degrees=True)
    follower = SO100Follower(config)
    follower.connect(calibrate=False)
    if not follower.is_connected:
        print("Failed to connect to SO-100 Follower arm. Please check the connection.")
        return
    print("SO-100 Follower arm connected successfully.")
    follower.bus.disable_torque() # Disable torque to be able to move the arm freely

    # Define initial joint positions (all zeros for now)
    joint_positions = {}
    for joint in robot.joints:
        joint_positions[joint.name] = 0.0

    print(f"Available joints: {list(joint_positions.keys())}")

    try:
        running = True
        while running:
            # Check for commands from the queue
            try:
                command = listener.command_queue.get_nowait()
                logging.info(f"Processing command: {command}")

                if command == CMD_RECORD:
                    logging.info("Starting recording...")
                    listener.is_recording = True
                    listener.recorded_data = []  # Clear previous recording
                    # Disable torque for recording (allow manual movement)
                    follower.bus.disable_torque()
                    logging.info("Torque disabled - arm can be moved manually")
                elif command == CMD_STOP:
                    logging.info("Stopping recording...")
                    listener.is_recording = False
                    logging.info(f"Recorded {len(listener.recorded_data)} data points")
                    # Disable torque after recording (allow manual movement)
                    follower.bus.disable_torque()
                    logging.info("Torque disabled - arm can be moved manually")
                elif command == CMD_REPLAY:
                    logging.info("Starting replay...")
                    logging.info(f"Replaying {len(listener.recorded_data)} data points")
                    listener.is_replaying = True
                    listener.replay_index = 0
                    # Enable torque for replay (robot will control movement)
                    follower.bus.enable_torque()
                    logging.info("Torque enabled - robot will control movement during replay")
                else:
                    logging.warning(f"Unknown command: {command}")

            except queue.Empty:
                # No commands in queue, continue with normal operation
                pass
            except Exception as e:
                logging.error(f"Error processing command: {e}")

            # Show recording status
            if listener.is_recording:
                print(f"\rRecording... {len(listener.recorded_data)} points captured", end="", flush=True)
            elif listener.is_replaying:
                progress = (listener.replay_index / len(listener.recorded_data)) * 100 if listener.recorded_data else 0
                print(f"\rReplaying... {listener.replay_index}/{len(listener.recorded_data)} ({progress:.1f}%)", end="", flush=True)
            else:
                print(f"\rIdle - {len(listener.recorded_data)} points recorded", end="", flush=True)

            if not running:
                break

            # Read and publish camera frame
            try:
                frame = camera.async_read(timeout_ms=200)
                if frame is not None:
                    img_msg = RawImage(
                        data=frame.tobytes(),
                        width=frame.shape[1],
                        height=frame.shape[0],
                        step=frame.shape[1] * 3,
                        encoding="rgb8",
                    )
                    image_channel.log(img_msg)
            except Exception as e:
                logging.warning(f"Error reading camera frame: {e}")

            # Read actual joint angles from follower (in degrees)
            try:
                obs = follower.get_observation()
                # print(obs)

                #TODO: The URDF 0 position does not match the calibrated arm 0 position
                # as described in the lerobot documentation.
                joint_positions["1"] = math.radians(obs.get("shoulder_pan.pos", 0.0)) * -1.
                joint_positions["2"] = math.radians(obs.get("shoulder_lift.pos", 0.0))
                joint_positions["3"] = math.radians(obs.get("elbow_flex.pos", 0.0))
                joint_positions["4"] = math.radians(obs.get("wrist_flex.pos", 0.0)-30)
                joint_positions["5"] = math.radians(obs.get("wrist_roll.pos", 0.0))
                # Convert gripper percent (0-100) to radians (0 to pi)
                joint_positions["6"] = ((obs.get("gripper.pos", 0.0) -10)/ 100.0) * math.pi

                # Record data if recording is active
                if listener.is_recording:
                    timestamp = time.time()
                    recorded_point = {
                        'timestamp': timestamp,
                        'joint_positions': joint_positions.copy(),
                        'observation': obs.copy()
                    }
                    listener.recorded_data.append(recorded_point)

                # Replay recorded actions if replay is active
                if listener.is_replaying and listener.replay_index < len(listener.recorded_data):
                    recorded_point = listener.recorded_data[listener.replay_index]
                    recorded_obs = recorded_point['observation']

                    # Convert recorded observation to action format for send_action
                    action = {}
                    for key, value in recorded_obs.items():
                        if key.endswith('.pos'):
                            action[key] = value

                    # Send the recorded action to the robot
                    try:
                        sent_action = follower.send_action(action)
                        logging.debug(f"Replay step {listener.replay_index}: sent action {sent_action}")
                    except Exception as e:
                        logging.error(f"Error sending replay action: {e}")

                    listener.replay_index += 1

                    # Check if replay is complete
                    if listener.replay_index >= len(listener.recorded_data):
                        logging.info("Replay completed!")
                        listener.is_replaying = False
                        # Disable torque after replay (allow manual movement)
                        follower.bus.disable_torque()
                        logging.info("Torque disabled - replay complete, arm can be moved manually")

                # print(f"Joint positions: {joint_positions}")

                # Compute forward kinematics with updated joint positions
                fk_poses = robot.link_fk(cfg=joint_positions)

                transforms = []
                # World -> Base
                transforms.append(
                    FrameTransform(
                        parent_frame_id=WORLD_FRAME_ID,
                        child_frame_id=BASE_FRAME_ID,
                        translation=Vector3(x=0.0, y=0.0, z=0.0),
                        rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                    )
                )
                # Per-joint transforms
                for joint in robot.joints:
                    parent_link = joint.parent
                    child_link = joint.child
                    T_parent = fk_poses[robot.link_map[parent_link]]
                    T_child = fk_poses[robot.link_map[child_link]]
                    # Local transform from parent->child
                    T_local = np.linalg.inv(T_parent) @ T_child
                    trans = T_local[:3, 3]
                    quat = rot_matrix_to_quat(T_local[:3, :3])
                    transforms.append(
                        FrameTransform(
                            parent_frame_id=parent_link,
                            child_frame_id=child_link,
                            translation=Vector3(x=float(trans[0]), y=float(trans[1]), z=float(trans[2])),
                            rotation=Quaternion(x=float(quat[0]), y=float(quat[1]), z=float(quat[2]), w=float(quat[3]))
                        )
                    )

                foxglove.log(
                    "/tf",
                    FrameTransforms(transforms=transforms)
                )
            except Exception as e:
                logging.warning(f"Error processing robot data: {e}")

            time.sleep(1.0 / RATE_HZ)

    except KeyboardInterrupt:
        print("\nShutting down Foxglove viewer...")
    except Exception as e:
        print(f"\nError occurred: {e}")
        logging.error(f"Unexpected error: {e}")
    finally:
        print("Cleaning up resources...")
        try:
            server.stop()
        except Exception as e:
            logging.error(f"Error stopping server: {e}")

        try:
            follower.disconnect()
        except Exception as e:
            logging.error(f"Error disconnecting follower: {e}")

        try:
            camera.disconnect()
        except Exception as e:
            logging.error(f"Error disconnecting camera: {e}")

        #writer.close()
        print("Cleanup complete.")

if __name__ == "__main__":
    main()

