"""Turn the foxglove.Event messages in a recording into Foxglove events.

Reads the data through the Foxglove API, so it works on Foxglove-hosted and
bring-your-own-storage sites alike. Shared by the local CLI and the Lambda
worker so there is one implementation.
"""

import io
import os
from datetime import timezone

import requests
from mcap.reader import make_reader
from mcap_protobuf.decoder import DecoderFactory

API = os.environ.get("FOXGLOVE_API", "https://api.foxglove.dev/v1")
EVENT_SCHEMA = "foxglove.Event"
SOURCE_KEY = "source_recording_id"
EVENT_KEY = "source_event_key"
PAGE = 2000  # maximum limit for GET /events


def _headers(api_key):
    return {"Authorization": f"Bearer {api_key}"}


def rfc3339(ts):
    return ts.ToDatetime(tzinfo=timezone.utc).isoformat().replace("+00:00", "Z")


def event_topics(recording_id, api_key):
    """Topics carrying foxglove.Event, selected by schema name, not topic name."""
    r = requests.get(
        f"{API}/data/topics", headers=_headers(api_key), timeout=30,
        params={"recordingId": recording_id, "includeSchemas": "true"},
    )
    r.raise_for_status()
    return [t["topic"] for t in r.json() if t.get("schemaName") == EVENT_SCHEMA]


def fetch_events_mcap(recording_id, topics, api_key):
    """A small MCAP holding only the event topics.

    The signed link expires 15 seconds after it is issued, so request it and
    download it in the same step. A retry requests a fresh link.
    """
    r = requests.post(
        f"{API}/data/stream", headers=_headers(api_key), timeout=30,
        json={"recordingId": recording_id, "topics": topics, "outputFormat": "mcap"},
    )
    r.raise_for_status()
    data = requests.get(r.json()["link"], timeout=60)  # signed: no auth header
    data.raise_for_status()
    return make_reader(io.BytesIO(data.content), decoder_factories=[DecoderFactory()])


def keyed_events(messages):
    """Pair each event message with a key that is stable across runs.

    The stream endpoint regenerates the MCAP on every call, so a message's
    position in it is not a stable identity: two events with the same log time
    can come back in either order. The key is topic, log time, and an
    occurrence count, assigned after sorting by content, so the same message
    gets the same key on every run. Messages that tie on all three are
    byte-identical, so swapping them changes nothing.
    """
    ordered = sorted(messages, key=lambda m: (m[1].topic, m[2].log_time, m[2].data))
    seen, keyed = {}, []
    for _, channel, message, proto in ordered:
        base = f"{channel.topic}:{message.log_time}"
        n = seen.get(base, 0)
        seen[base] = n + 1
        keyed.append((f"{base}:{n}", proto))
    return keyed


def existing_keys(recording_id, device_id, api_key):
    """Keys of the events already created for this recording, across all pages.

    GET /events has no recording filter, so this is a metadata query scoped to
    the device.
    """
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
        seen.update((e.get("metadata") or {}).get(EVENT_KEY) for e in page)
        if len(page) < PAGE:
            return seen
        offset += PAGE


def run(recording_id, device_id, api_key):
    """Returns the number of events created. Idempotent per event: a retry after
    a partial failure creates only the events that are missing."""
    topics = event_topics(recording_id, api_key)
    if not topics:
        print(f"{recording_id}: no {EVENT_SCHEMA} topic, nothing to do")
        return 0
    print(f"{recording_id}: event topics {topics}")

    reader = fetch_events_mcap(recording_id, topics, api_key)
    messages = keyed_events(reader.iter_decoded_messages(topics=topics))
    existing = existing_keys(recording_id, device_id, api_key)
    created = 0
    for key, proto in messages:
        if key in existing:
            continue
        metadata = {kv.key: kv.value for kv in proto.metadata}
        r = requests.post(
            f"{API}/events", headers=_headers(api_key), timeout=30,
            json={"deviceId": device_id,
                  "start": rfc3339(proto.start_time),
                  "end": rfc3339(proto.end_time),
                  "metadata": {**metadata, SOURCE_KEY: recording_id,
                               EVENT_KEY: key}},
        )
        r.raise_for_status()
        created += 1
        print(f"  created {r.json()['id']} {metadata}")
    print(f"{recording_id}: {created} new events")
    return created
