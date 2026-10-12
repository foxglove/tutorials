#!/usr/bin/env python3
"""Run the ingestion pipeline once, by hand.

    python ingest_events.py <bucket> <key> <recording_id> <device_id>

Same code the Lambda worker runs - both call ingest.run().
"""

import os
import sys

import ingest


def main():
    if len(sys.argv) != 5:
        print(__doc__)
        return 2
    bucket, key, recording_id, device_id = sys.argv[1:5]
    ingest.run(bucket=bucket, key=key, recording_id=recording_id,
               device_id=device_id, api_key=os.environ["FOXGLOVE_API_KEY"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
