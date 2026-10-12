#!/usr/bin/env python3
"""Generate an MCAP containing foxglove.Event messages.

    python write_events.py [outfile]

Events are stamped at the current time, so every run produces a file with
different content. That matters: Foxglove rejects a recording whose content
hash matches one already imported for the same device, so re-uploading an
identical file is silently dropped rather than indexed.
"""

import sys
import time

import foxglove
from foxglove.channels import EventChannel
from foxglove.messages import Event, Timestamp, KeyValuePair

EVENTS = [
    ("pick_attempt", "grasp"),
    ("dropoff", "place"),
    ("fault", "error"),
]


def main(path="sample_events.mcap", base=None):
    base = int(time.time()) if base is None else int(base)
    writer = foxglove.open_mcap(path, allow_overwrite=True)
    channel = EventChannel("/events")

    for i, (name, kind) in enumerate(EVENTS):
        start = base + i * 10
        channel.log(
            Event(
                start_time=Timestamp(sec=start, nsec=0),
                end_time=Timestamp(sec=start + 4, nsec=500_000_000),
                metadata=[
                    KeyValuePair(key="name", value=name),
                    KeyValuePair(key="kind", value=kind),
                    KeyValuePair(key="run", value=str(base)),
                ],
            ),
            log_time=start * 1_000_000_000,
        )

    writer.close()
    print(f"wrote {path} with {len(EVENTS)} events at t={base}")


if __name__ == "__main__":
    main(*sys.argv[1:2])
