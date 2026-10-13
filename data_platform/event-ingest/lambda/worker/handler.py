"""SQS worker. Reads the recording's event topics through the Foxglove API and
creates the events."""

import json
import os
import time

import boto3

import ingest

SECRET_TTL_SECONDS = int(os.environ.get("SECRET_TTL_SECONDS", "300"))

_api_key = None
_api_key_fetched_at = 0.0


def api_key():
    """Cached with a TTL, like the receiver's signing token, so a rotated key
    takes effect without redeploying the worker."""
    global _api_key, _api_key_fetched_at
    if _api_key is None or (time.time() - _api_key_fetched_at) > SECRET_TTL_SECONDS:
        _api_key = boto3.client("secretsmanager").get_secret_value(
            SecretId=os.environ["FOXGLOVE_API_KEY_SECRET_ARN"])["SecretString"].strip()
        _api_key_fetched_at = time.time()
    return _api_key


def handler(event, context):
    for record in event.get("Records", []):
        msg = json.loads(record["body"])
        recording_id = msg.get("recordingId")

        # The receiver drops these already. This catches messages queued by an
        # older receiver that didn't, which would otherwise retry into the DLQ.
        if not msg.get("deviceId"):
            print(f"{recording_id}: no device on recording, skipping")
            continue

        ingest.run(recording_id=recording_id, device_id=msg["deviceId"],
                   api_key=api_key())
