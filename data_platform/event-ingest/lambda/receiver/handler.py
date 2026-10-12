"""Webhook receiver. Verifies the signature, enqueues, returns fast.

Foxglove treats a response slower than 5 seconds as a timeout and may retry,
so this function must not read the recording or call the Foxglove API.

Every path logs its outcome. A silent handler is impossible to debug from
CloudWatch, because a rejected request and an ignored one look identical.
"""

import base64
import hashlib
import hmac
import json
import os
import time

import boto3

QUEUE_URL = os.environ["QUEUE_URL"]
WEBHOOK_SECRET_ARN = os.environ["WEBHOOK_SECRET_ARN"]
SECRET_TTL_SECONDS = int(os.environ.get("SECRET_TTL_SECONDS", "300"))

sqs = boto3.client("sqs")
_secret = None
_secret_fetched_at = 0.0


def webhook_secret(force_refresh=False):
    """Cached with a TTL, and refreshable on demand.

    Caching forever means a rotated token is not picked up until the container
    is recycled, which tempts you into replacing the function to force a cold
    start - and that destroys its resource-based policy.
    """
    global _secret, _secret_fetched_at
    expired = (time.time() - _secret_fetched_at) > SECRET_TTL_SECONDS
    if force_refresh or _secret is None or expired:
        _secret = boto3.client("secretsmanager").get_secret_value(
            SecretId=WEBHOOK_SECRET_ARN)["SecretString"].strip()
        _secret_fetched_at = time.time()
    return _secret


def raw_body(event):
    body = event.get("body") or ""
    if event.get("isBase64Encoded"):
        return base64.b64decode(body)
    return body.encode("utf-8")


def signature_ok(raw, provided):
    """Verify, retrying once against a freshly fetched secret.

    Handles token rotation without needing a cold start.
    """
    for force in (False, True):
        expected = hmac.new(webhook_secret(force).encode(), raw, hashlib.sha256).hexdigest()
        if hmac.compare_digest(expected, provided):
            return True
    return False


def handler(event, context):
    raw = raw_body(event)
    headers = {k.lower(): v for k, v in (event.get("headers") or {}).items()}

    # Hash the raw bytes. Re-serializing parsed JSON changes them and the
    # digest will never match.
    if not signature_ok(raw, headers.get("fg-webhook-signature", "")):
        print("rejected: signature mismatch (checked cached and fresh secret)")
        return {"statusCode": 401, "body": "invalid signature"}

    payload = json.loads(raw)
    event_type = payload.get("type")

    if event_type != "recording.imported":
        print(f"ignored: event type {event_type!r}, signature was valid")
        return {"statusCode": 200, "body": "ignored"}

    rec = payload.get("recording") or {}
    loc = rec.get("location") or {}
    recording_id = rec.get("id")

    if not loc.get("bucket") or not loc.get("path"):
        # Query-optimized site, or not imported yet: no object to read.
        print(f"{recording_id}: no location on recording, nothing to do")
        return {"statusCode": 200, "body": "no location"}

    device_id = (rec.get("device") or {}).get("id")
    sqs.send_message(QueueUrl=QUEUE_URL, MessageBody=json.dumps({
        "recordingId": recording_id,
        "deviceId": device_id,
        "bucket": loc["bucket"],
        "key": loc["path"],
    }))
    print(f"{recording_id}: queued bucket={loc['bucket']} key={loc['path']} device={device_id}")
    return {"statusCode": 200, "body": "queued"}
