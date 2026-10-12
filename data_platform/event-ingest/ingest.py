"""Read foxglove.Event messages from an MCAP in S3 and create Foxglove events.

Shared by the local CLI and the Lambda worker so there is one implementation.
"""

import io
import os
from datetime import timezone

import boto3
import requests
from mcap.reader import make_reader
from mcap_protobuf.decoder import DecoderFactory

API = os.environ.get("FOXGLOVE_API", "https://api.foxglove.dev/v1")
EVENT_SCHEMA = "foxglove.Event"
SOURCE_KEY = "source_recording_id"
INDEX_KEY = "source_event_index"
PAGE = 1000


class S3File(io.RawIOBase):
    """Seekable file-like over an S3 object, backed by HTTP range requests.

    The MCAP reader asks the file two separate questions: the summary section
    at the end, then the chunks holding the topic we want. A streaming body
    from get_object() only goes forward, so the second query fails with
    "cannot use more than one query against a non-seeking data source".
    """

    def __init__(self, client, bucket, key):
        self._client, self._bucket, self._key = client, bucket, key
        self._pos = 0
        self._size = client.head_object(Bucket=bucket, Key=key)["ContentLength"]

    def readable(self): return True
    def seekable(self): return True
    def tell(self): return self._pos

    def seek(self, offset, whence=io.SEEK_SET):
        if whence == io.SEEK_SET:
            self._pos = offset
        elif whence == io.SEEK_CUR:
            self._pos += offset
        elif whence == io.SEEK_END:
            self._pos = self._size + offset
        self._pos = max(0, min(self._pos, self._size))
        return self._pos

    def readinto(self, b):
        if self._pos >= self._size:
            return 0
        end = min(self._pos + len(b), self._size) - 1
        body = self._client.get_object(
            Bucket=self._bucket, Key=self._key, Range=f"bytes={self._pos}-{end}"
        )["Body"].read()
        n = len(body)
        b[:n] = body
        self._pos += n
        return n


def _headers(api_key):
    return {"Authorization": f"Bearer {api_key}"}


def rfc3339(ts):
    return ts.ToDatetime().replace(tzinfo=timezone.utc).isoformat().replace("+00:00", "Z")


def existing_indexes(recording_id, device_id, api_key):
    """Indexes of the events already created for this recording, across all pages."""
    seen, offset = set(), 0
    while True:
        r = requests.get(
            f"{API}/events", headers=_headers(api_key), timeout=30,
            params={"deviceId": device_id, "queryFields": "metadata",
                    "query": f"{SOURCE_KEY}:{recording_id}",
                    "limit": PAGE, "offset": offset},
        )
        r.raise_for_status()
        page = r.json()
        seen.update(e.get("metadata", {}).get(INDEX_KEY) for e in page)
        if len(page) < PAGE:
            return seen
        offset += PAGE


def open_recording(bucket, key, client=None):
    raw = S3File(client or boto3.client("s3"), bucket, key)
    return make_reader(io.BufferedReader(raw, buffer_size=1024 * 1024),
                       decoder_factories=[DecoderFactory()])


def event_topics(reader):
    summary = reader.get_summary()
    return [ch.topic for ch in summary.channels.values()
            if summary.schemas[ch.schema_id].name == EVENT_SCHEMA]


def run(bucket, key, recording_id, device_id, api_key, s3_client=None):
    """Returns the number of events created. Idempotent per event: a retry after
    a partial failure creates only the events that are missing."""
    reader = open_recording(bucket, key, s3_client)
    topics = event_topics(reader)
    if not topics:
        print(f"{recording_id}: no {EVENT_SCHEMA} channel, nothing to do")
        return 0
    print(f"{recording_id}: event topics {topics}")

    existing = existing_indexes(recording_id, device_id, api_key)
    created = 0
    for i, (_, _, _, proto) in enumerate(reader.iter_decoded_messages(topics=topics)):
        if str(i) in existing:
            continue
        metadata = {kv.key: kv.value for kv in proto.metadata}
        r = requests.post(
            f"{API}/events", headers=_headers(api_key), timeout=30,
            json={"deviceId": device_id,
                  "start": rfc3339(proto.start_time),
                  "end": rfc3339(proto.end_time),
                  "metadata": {**metadata, SOURCE_KEY: recording_id,
                               INDEX_KEY: str(i)}},
        )
        r.raise_for_status()
        created += 1
        print(f"  created {r.json()['id']} {metadata}")
    print(f"{recording_id}: {created} new events")
    return created
