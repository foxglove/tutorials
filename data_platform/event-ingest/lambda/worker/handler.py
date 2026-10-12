"""SQS worker. Reads the recording from the bucket and creates the events."""

import json
import os

import boto3

import ingest

_api_key = None


def api_key():
    global _api_key
    if _api_key is None:
        _api_key = boto3.client("secretsmanager").get_secret_value(
            SecretId=os.environ["FOXGLOVE_API_KEY_SECRET_ARN"])["SecretString"].strip()
    return _api_key


def handler(event, context):
    for record in event.get("Records", []):
        msg = json.loads(record["body"])
        recording_id = msg.get("recordingId")

        if not msg.get("deviceId"):
            # Retrying will not attach a device. Drop rather than cycle to the DLQ.
            print(f"{recording_id}: no device on recording, skipping")
            continue

        ingest.run(bucket=msg["bucket"], key=msg["key"],
                   recording_id=recording_id, device_id=msg["deviceId"],
                   api_key=api_key())
