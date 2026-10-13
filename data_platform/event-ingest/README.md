---
title: Create Foxglove events from foxglove.Event messages in MCAP
short_description: A webhook-driven pipeline that reads each newly imported recording through the Foxglove API and turns its foxglove.Event messages into Foxglove events.
---

# Create Foxglove events from `foxglove.Event` messages in MCAP

If your stack already publishes `foxglove.Event` messages into its MCAP files, those
messages are just data on a topic. Foxglove does not read them during indexing, so
nothing appears on the timeline.

This pipeline fires on the `recording.imported` webhook, asks the Foxglove API for just
the recording's event topics, and creates one Foxglove event per message it finds.

```
recording.imported webhook
  -> receiver     verify HMAC, enqueue, return 200
  -> queue (+ dead-letter queue)
  -> worker       GET /data/topics
                  POST /data/stream, download the signed link
                  GET + POST /events
```

Nothing here depends on a particular cloud: the worker only talks to the Foxglove API.
`ingest.py` is the whole pipeline, and you can run it by hand with no cloud at all. The
deployment in `terraform/` is one worked example on AWS (API Gateway and Lambda for the
receiver, SQS for the queue, Lambda for the worker). On GCP, Cloud Run and Pub/Sub fill
the same roles, and on Azure, Functions and Queue Storage do.

It works on Foxglove-hosted and [bring-your-own-storage](https://docs.foxglove.dev/docs/data/primary-sites/foxglove-managed-setup)
sites with one Foxglove API key and no access to your storage bucket. The worker
downloads only a small MCAP holding the event topics, not the whole recording.

## Dependencies

Python 3.12+, and an AWS account if you want to deploy the AWS example. Install into a virtual
environment; many systems refuse a system-wide `pip install`, and `build.sh` needs `pip`
too.

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

That pulls `mcap`, `mcap-protobuf-support`, `requests`, `boto3`, and `foxglove-sdk`
(the last only to generate the sample recording).

Deploying also needs the [AWS CLI](https://docs.aws.amazon.com/cli/),
[Terraform](https://developer.hashicorp.com/terraform/install) 1.5+, and an S3 bucket for
Terraform state.

## Run it once, locally

You need a Foxglove API key, created by an organization admin under **Settings → API
keys**, with these capabilities:

- `data.topics.list`, to find the recording's event topics
- `data.stream`, to fetch those topics
- `events.list`, to check which events already exist
- `events.create`, to create the new ones
- `recordings.list`, only for looking up recording IDs by hand in step 3

**1. Generate a sample recording.**

```bash
python write_events.py
```

Events are stamped at the current time, so every run produces a content-distinct file.
That matters; see the note on duplicates below.

**2. Upload it to a device** in Foxglove, the same way you upload any recording.

On a bring-your-own-storage site, a file dropped straight into the bucket carries no
identity: without [object metadata](https://docs.foxglove.dev/docs/data/primary-sites/manage-data#object-metadata)
it fails to index with `No projectId could be determined from the provided identifiers`,
and no webhook ever fires. Set the metadata in the same operation as the write. On AWS:

```bash
aws s3 cp sample_events.mcap s3://<bucket>/<device>/sample_events.mcap \
  --metadata foxglove_device_name=<device>,foxglove_project_id=<project-id>
```

On Google Cloud:

```bash
gcloud storage cp sample_events.mcap gs://<bucket>/<device>/sample_events.mcap \
  --custom-metadata=foxglove_device_name=<device>,foxglove_project_id=<project-id>
```

The [Cloud CLI uploads](https://docs.foxglove.dev/docs/data/primary-sites/manage-data#cloud-cli-uploads)
docs cover Azure too.

**3. Wait for indexing**, which is asynchronous and gives no progress feedback. Then
grab the recording ID and device ID:

```bash
export FOXGLOVE_API_KEY="fox_sk_..."
export FOXGLOVE_API="https://api.foxglove.dev/v1"

curl -s -H "Authorization: Bearer $FOXGLOVE_API_KEY" \
  "$FOXGLOVE_API/recordings?limit=3&sortBy=createdAt&sortOrder=desc"
```

**4. Run the pipeline.**

```bash
python ingest_events.py <recording-id> <device-id>
```

Run it twice. The second run prints `0 new events`.

## Deploy it

On any platform, the deployment needs a secret store for the API key and signing token, a
queue visibility timeout well above the worker's timeout, a cap on worker concurrency, and
a dead-letter queue with an alarm. The steps below deploy the AWS example.

Create the webhook **before** deploying. There's a circular dependency: Terraform needs
the signing token, Foxglove only reveals it at creation, and creating a webhook needs a
URL Terraform hasn't produced yet. Create it against a placeholder URL, take the token,
deploy, then edit the webhook to the real endpoint.

**1. Put the secrets in Secrets Manager.** They are deliberately created outside
Terraform, because a secret passed as a variable ends up in state in plaintext.

```bash
aws secretsmanager create-secret --name foxglove/api-key \
  --secret-string "$FOXGLOVE_API_KEY" --query ARN --output text
aws secretsmanager create-secret --name foxglove/webhook-token \
  --secret-string "<token from the webhook>" --query ARN --output text
```

**2. Build and apply.**

```bash
./build.sh
cd terraform
cp backend.tfvars-example backend.tfvars       # your state bucket
cp terraform.tfvars-example terraform.tfvars   # your region and the two ARNs
terraform init -backend-config=backend.tfvars
terraform plan
terraform apply
```

The Terraform rejects the placeholder secret ARNs in `terraform.tfvars-example`, so
`terraform plan` fails until you've filled in your own.

**3. Point the webhook** at the `webhook_url` output, subscribed to `recording.imported`:

```bash
terraform output -raw webhook_url
```

## Verify it

Start with a health check that doesn't involve Foxglove:

```bash
curl -i -X POST "$(terraform output -raw webhook_url)" \
  -H 'content-type: application/json' -d '{"type":"ping"}'
```

`401 invalid signature` is the success case. That one call proves routing, invoke
permission, the secret read and HMAC verification all work. If a later upload creates no
events, look at the webhook configuration or the worker, not the receiver.

A 500 *with* Lambda logs is a code error. A 500 with no logs means the function never
ran, so it's routing or invoke permission.

Then generate a fresh sample with `python ../write_events.py` (you're still in
`terraform/`), upload it, and watch the worker log for `3 new events`:

```bash
aws logs tail "$(terraform output -raw worker_log_group)" --follow
```

## Tests

```bash
python3 test_pipeline.py
```

Sixteen offline tests: the pipeline end to end against a fake Foxglove API, including a
retry after a partial failure and event keys that survive the stream reordering messages,
signature verification including a tampered body, and the handling of a recording with no
device. No AWS, no network.

## Layout

```
ingest.py            the pipeline. One implementation, used by the CLI and the worker.
ingest_events.py     run it once by hand
write_events.py      generate a sample recording
test_pipeline.py     offline tests
build.sh             package the Lambdas (AWS example)
lambda/              receiver and worker handlers (AWS example)
terraform/           AWS example: API Gateway, Lambdas, SQS + DLQ, alarm, IAM
```

## Notes

- **Detect the topic by schema, not by name.** The pipeline looks for the
  `foxglove.Event` schema, so `/events`, `/robot/events` and `/diagnostics/markers` all
  work with no configuration.

- **The signed link expires 15 seconds after it is issued.** `POST /data/stream` returns
  a link, not data. Request it and download it in the same step; a retry requests a new
  link. Don't send the API key to the signed link.

- **Idempotency is per event.** Every created event carries `source_recording_id` and
  `source_event_key` in its metadata. The key is topic, log time and an occurrence count,
  not the message's position, because the stream endpoint regenerates the MCAP on every
  call and same-timestamp events can come back in either order. Each run lists the events
  already created for the recording and writes only the missing keys, so a retry after a
  partial failure fills in the gaps instead of skipping the recording. Webhook retries and
  manual retries are safe when they run one after another. Two deliveries of the same
  recording that run *at the same time* can both pass the check and create duplicates; a
  FIFO queue with the recording ID as `MessageDeduplicationId` closes that gap.

- **Duplicate uploads are silently dropped.** Foxglove hashes content and refuses a
  recording whose hash matches one already imported for the same device. Renaming the
  object doesn't help. The recording never appears, no webhook fires, and the only trace
  is a line under **Import Errors** in the app, which is also where quarantined files
  go, and the first place to look when a recording seems to have vanished.

- **No device, no events.** `POST /events` requires one. The receiver logs and drops a
  recording without a device rather than queueing it, since a retry can't add one.

- **Keeping data in your own account.** On a bring-your-own-storage site, the worker could
  read the MCAP straight from your bucket instead, which keeps the data path inside your
  cloud account at the cost of a storage permission grant and BYOS-only support.

- **Protobuf only.** The worker reads `schemaEncoding` from `GET /data/topics` and skips
  any JSON-encoded `foxglove.Event` topic with a log line, so the protobuf events in the
  same recording are still created. Supporting JSON events needs a JSON decoder and the
  `sec`/`nsec` field names rather than `seconds`/`nanos`.

- **Receiver deploys cause a few seconds of 500s (AWS example).** Replacing a Lambda function destroys
  its resource-based policy, so `aws_lambda_permission.apigw` is set to be recreated
  whenever the receiver changes. Between the two steps API Gateway can't invoke the
  function. Foxglove retries webhook deliveries, so nothing is lost. You also don't need a
  cold start to pick up a rotated token: the receiver caches its secret with a TTL and
  re-reads it on a signature mismatch.
