---
title: Create Foxglove events from foxglove.Event messages in MCAP
short_description: A webhook-driven pipeline that reads each newly imported recording out of your own bucket and turns its foxglove.Event messages into Foxglove events.
---

# Create Foxglove events from `foxglove.Event` messages in MCAP

If your stack already publishes `foxglove.Event` messages into its MCAP files, those
messages are just data on a topic — Foxglove does not read them during indexing, so
nothing appears on the timeline.

This pipeline closes that gap. It fires on the `recording.imported` webhook, range-reads
the new recording directly from your own bucket, and creates one Foxglove event per
message it finds.

```
recording.imported webhook
  -> API Gateway -> receiver Lambda    verify HMAC, enqueue, return 200
  -> SQS (+ DLQ)
  -> worker Lambda                     range-read the MCAP, create events
```

Assumes Foxglove Cloud with [bring-your-own-storage](https://docs.foxglove.dev/docs/data/primary-sites/foxglove-managed-setup).
The recording is already in storage you control, so the worker reads it in place rather
than asking Foxglove to stream it back. It never downloads the file: against the sample
recording it issues four HTTP range requests and transfers only the bytes it needs.

## Dependencies

Python 3.12+, and an AWS account if you want to deploy it.

```bash
pip install -r requirements.txt
```

That pulls `mcap`, `mcap-protobuf-support`, `requests`, `boto3`, and `foxglove-sdk`
(the last only to generate the sample recording).

Deploying also needs the [AWS CLI](https://docs.aws.amazon.com/cli/) and
[Terraform](https://developer.hashicorp.com/terraform/install).

## Run it once, locally

You need a BYOS primary site and a Foxglove API key with event write access.

**1. Generate a sample recording.**

```bash
python write_events.py
```

Events are stamped at the current time, so every run produces a content-distinct file.
That matters — see the note on duplicates below.

**2. Upload it with identity metadata.**

```bash
aws s3 cp sample_events.mcap s3://<bucket>/<device>/sample_events.mcap \
  --metadata foxglove_device_name=<device>,foxglove_project_id=<project-id>
```

The metadata is required. A file dropped into a BYOS bucket carries no identity, so
without it the recording fails to index with `No projectId could be determined from the
provided identifiers`, and no webhook ever fires. `foxglove_device_name` creates the
device if it doesn't exist. Set the metadata in the same operation as the write — added
afterwards, Foxglove may read the object first.

For production, set an ingest path policy on the site so uploaders don't have to
remember.

**3. Wait for indexing**, which is asynchronous and gives no progress feedback. Then
grab the recording ID, device ID and `location`:

```bash
export FOXGLOVE_API_KEY="fox_sk_..."
export FOXGLOVE_API="https://api.foxglove.dev/v1"

curl -s -H "Authorization: Bearer $FOXGLOVE_API_KEY" \
  "$FOXGLOVE_API/recordings?limit=3&sortBy=createdAt&sortOrder=desc"
```

**4. Run the pipeline.**

```bash
python ingest_events.py <bucket> <key> <recording-id> <device-id>
```

Run it twice. The second run prints `0 new events`.

## Deploy it

Create the webhook **before** deploying. There's a circular dependency: Terraform needs
the signing token, Foxglove only reveals it at creation, and creating a webhook needs a
URL Terraform hasn't produced yet. Create it against a placeholder URL, take the token,
deploy, then edit the webhook to the real endpoint.

**1. Put the secrets in Secrets Manager.** They are deliberately created outside
Terraform — a secret passed as a variable ends up in state in plaintext.

```bash
aws secretsmanager create-secret --name foxglove/api-key \
  --secret-string "fox_sk_..."
aws secretsmanager create-secret --name foxglove/webhook-token \
  --secret-string "<token from the webhook>"
```

**2. Build and apply.**

```bash
./build.sh
cd terraform
cp terraform.tfvars-example terraform.tfvars   # then edit it
terraform init
terraform plan
terraform apply
```

The Terraform rejects the placeholder values in `terraform.tfvars-example`, so
`terraform plan` fails until you've filled in your bucket and secret ARNs.

Run the worker in the **same region as the recordings bucket**.

**3. Point the webhook** at the `webhook_url` output, subscribed to `recording.imported`.

## Verify it

Start with a health check that doesn't involve Foxglove:

```bash
curl -i -X POST https://<api-id>.execute-api.<region>.amazonaws.com/webhook \
  -H 'content-type: application/json' -d '{"type":"ping"}'
```

`401 invalid signature` is the success case. That one call proves routing, invoke
permission, the secret read and HMAC verification, so every later problem splits cleanly
into an AWS-side one and a Foxglove-side one.

A 500 *with* Lambda logs is a code error. A 500 with no logs means the function never
ran — routing or invoke permission.

Then upload a recording and watch the worker log. A run against the three-event sample
takes about 1.28s and peaks at 111 MB of a 512 MB allocation, with a 722 ms cold start.
Recording size barely moves those numbers.

## Tests

```bash
python3 test_pipeline.py
```

Twelve offline tests: the pipeline end to end against a fake S3 that only serves range
reads, signature verification including a tampered body, and the worker's handling of a
recording with no device. No AWS, no network.

## Layout

```
ingest.py            the pipeline. One implementation, used by the CLI and the worker.
ingest_events.py     run it once by hand
write_events.py      generate a sample recording
test_pipeline.py     offline tests
build.sh             package the Lambdas
lambda/              receiver and worker handlers
terraform/           API Gateway, Lambdas, SQS + DLQ, alarm, IAM
```

## Notes

- **Detect the topic by schema, not by name.** The pipeline looks for the
  `foxglove.Event` schema, so `/events`, `/robot/events` and `/diagnostics/markers` all
  work with no configuration.

- **Seekable reads are required**, not an optimization. `get_object()["Body"]` is
  forward-only and the MCAP reader queries the file twice — the summary section, then the
  chunks holding the event topic. `ingest.py` wraps the object in a ~30-line file-like
  that implements `seek` and `readinto` over HTTP range requests.

- **Idempotency is per event.** Every created event carries `source_recording_id` and
  `source_event_index` in its metadata. Each run lists the events already created for the
  recording and writes only the missing indexes, so a retry after a partial failure fills
  in the gaps instead of skipping the recording. Webhook retries, manual retries and
  re-imports are all safe when they run one after another. Two deliveries of the same
  recording that run *at the same time* can both pass the check and create duplicates; a
  FIFO queue with the recording ID as `MessageDeduplicationId` closes that gap.

- **Duplicate uploads are silently dropped.** Foxglove hashes content and refuses a
  recording whose hash matches one already imported for the same device. Renaming the
  object doesn't help. The recording never appears, no webhook fires, and the only trace
  is a line under **Import Errors** in the app — which is also where quarantined files
  go, and the first place to look when a recording seems to have vanished.

- **No device, no events.** `POST /events` requires one. A recording without a device is
  skipped and logged rather than retried.

- **Archived objects.** A lifecycle policy that moves objects to a cold tier will break
  the range read. Handle or exclude them before running a backfill over old recordings.

- **Protobuf only.** A JSON-encoded `foxglove.Event` hits a different decoder and the
  `sec`/`nsec` field names rather than `seconds`/`nanos`. `schemaEncoding` from
  `GET /data/topics` tells you which you have.

- **Receiver deploys cause a few seconds of 500s.** Replacing a Lambda function destroys
  its resource-based policy, so `aws_lambda_permission.apigw` is set to be recreated
  whenever the receiver changes. Between the two steps API Gateway can't invoke the
  function. Foxglove retries webhook deliveries, so nothing is lost. You also don't need a
  cold start to pick up a rotated token: the receiver caches its secret with a TTL and
  re-reads it on a signature mismatch.
