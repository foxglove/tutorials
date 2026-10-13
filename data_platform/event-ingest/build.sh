#!/usr/bin/env bash
# Build the Lambda deployment packages. Run before terraform apply.
set -euo pipefail
cd "$(dirname "$0")"

rm -rf build
mkdir -p build/receiver build/worker

# Receiver needs nothing beyond the runtime - boto3 is provided by Lambda.
cp lambda/receiver/handler.py build/receiver/

# Worker gets the handler, the shared pipeline, and the MCAP stack. mcap pulls in
# lz4 and zstandard, which cover the stream endpoint's chunk compression.
# --platform/--only-binary keeps the wheels manylinux so they load on Lambda
# even when you build on a Mac.
cp lambda/worker/handler.py ingest.py build/worker/
pip install \
  --target build/worker \
  --platform manylinux2014_x86_64 \
  --implementation cp \
  --python-version 3.12 \
  --only-binary=:all: \
  --upgrade \
  mcap mcap-protobuf-support requests

find build -name '__pycache__' -type d -exec rm -rf {} + 2>/dev/null || true
echo "built: worker $(du -sh build/worker | cut -f1), receiver $(du -sh build/receiver | cut -f1)"
