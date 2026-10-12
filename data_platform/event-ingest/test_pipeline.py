"""Offline tests for the whole pipeline. No AWS, no network.

    python3 test_pipeline.py
"""
import base64, hashlib, hmac, importlib.util, io, json, os, sys, types
from datetime import datetime, timezone

HERE = os.path.dirname(os.path.abspath(__file__))
MCAP = os.path.join(HERE, "sample_events.mcap")
sys.path.insert(0, HERE)

# Generated rather than committed: a fixed sample file is a duplicate-upload trap.
# Pinned base time so the timestamp assertions below are deterministic.
BASE = 1760000000
import write_events
write_events.main(MCAP, base=BASE)

SIZE = os.path.getsize(MCAP)


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


# ---------------------------------------------------------------- fakes ----
class Resp:
    def __init__(self, p): self._p = p
    def raise_for_status(self): pass
    def json(self): return self._p


class FakeBody:
    def __init__(self, d): self._d = d
    def read(self): return self._d


class RangeOnlyS3:
    """Models S3: forward-only body, range reads only. A seekable stub would
    hide the bug this exists to catch."""
    def __init__(self): self.ranges = []
    def head_object(self, Bucket, Key): return {"ContentLength": SIZE}
    def get_object(self, Bucket, Key, Range=None):
        assert Range, "non-range read would fail against real S3"
        lo, hi = map(int, Range.replace("bytes=", "").split("-"))
        self.ranges.append((lo, hi))
        with open(MCAP, "rb") as f:
            f.seek(lo); return {"Body": FakeBody(f.read(hi - lo + 1))}


# ------------------------------------------------------------- ingest -----
import ingest  # noqa: E402

posted, existing = [], []
s3 = RangeOnlyS3()
ingest.requests = types.SimpleNamespace(
    get=lambda url, **kw: Resp(existing),
    post=lambda url, **kw: (posted.append(kw["json"]), Resp({"id": f"evt_{len(posted)}"}))[1],
)

n = ingest.run("bkt", "a/b.mcap", "rec_1", "dev_1", "key", s3_client=s3)
assert n == 3, n
expected_start = datetime.fromtimestamp(BASE, timezone.utc).isoformat().replace("+00:00", "Z")
assert posted[0]["start"] == expected_start, posted[0]
assert posted[0]["metadata"]["name"] == "pick_attempt"
assert posted[0]["metadata"]["source_recording_id"] == "rec_1"
print(f"1 ingest.run -> 3 events, {len(s3.ranges)} range reads, "
      f"{sum(h - l + 1 for l, h in s3.ranges)} bytes")

existing.append({"id": "evt_1"})
before = len(posted)
assert ingest.run("bkt", "a/b.mcap", "rec_1", "dev_1", "key", s3_client=s3) == 0
assert len(posted) == before
print("2 second run -> deduped, nothing created")

# ------------------------------------------------------------ handlers ----
SECRET, sent, fetches = "s3cr3t", [], []
def _sm_get(SecretId):
    fetches.append(SecretId)
    return {"SecretString": SECRET if "webhook" in SecretId else "fox_sk"}
sys.modules["boto3"] = types.SimpleNamespace(client=lambda svc, *a, **k: (
    types.SimpleNamespace(send_message=lambda QueueUrl, MessageBody: sent.append(json.loads(MessageBody)))
    if svc == "sqs" else
    types.SimpleNamespace(get_secret_value=_sm_get)))
os.environ.update(QUEUE_URL="q", WEBHOOK_SECRET_ARN="arn:secret:foxglove/webhook-token",
                  FOXGLOVE_API_KEY_SECRET_ARN="arn:secret:foxglove/api-key")

receiver = load("receiver_handler", f"{HERE}/lambda/receiver/handler.py")

def sign(raw): return hmac.new(SECRET.encode(), raw, hashlib.sha256).hexdigest()
def evt(payload, sig=None, b64=False):
    raw = json.dumps(payload).encode()
    return {"body": base64.b64encode(raw).decode() if b64 else raw.decode(),
            "isBase64Encoded": b64,
            "headers": {"FG-Webhook-Signature": sig if sig is not None else sign(raw)}}

IMPORTED = {"type": "recording.imported", "recording": {
    "id": "rec_1", "device": {"id": "dev_1"},
    "location": {"bucket": "bkt", "path": "a/b.mcap"}}}

cases = [
    ("valid signature",      evt(IMPORTED),                                200, 1),
    ("base64 body",          evt(IMPORTED, sig=sign(json.dumps(IMPORTED).encode()), b64=True), 200, 1),
    ("bad signature",        evt(IMPORTED, sig="deadbeef"),                401, 0),
    ("no signature header",  {**evt(IMPORTED), "headers": {}},             401, 0),
    ("other event type",     evt({"type": "ping"}),                        200, 0),
    ("no location",          evt({"type": "recording.imported",
                                  "recording": {"id": "rec_2"}}),          200, 0),
]
for i, (name, e, code, enqueued) in enumerate(cases, start=3):
    sent.clear()
    r = receiver.handler(e, None)
    assert r["statusCode"] == code, (name, r)
    assert len(sent) == enqueued, (name, sent)
    print(f"{i} receiver, {name} -> {code}, {len(sent)} enqueued")

# tampered body with the original signature
sent.clear()
tampered = json.loads(json.dumps(IMPORTED)); tampered["recording"]["location"]["bucket"] = "evil"
assert receiver.handler(evt(tampered, sig=sign(json.dumps(IMPORTED).encode())), None)["statusCode"] == 401
assert sent == []
print("9 receiver, tampered body -> 401, nothing enqueued")

worker = load("worker_handler", f"{HERE}/lambda/worker/handler.py")
calls = []
worker.ingest = types.SimpleNamespace(run=lambda **kw: calls.append(kw))

worker.handler({"Records": [{"body": json.dumps(
    {"recordingId": "rec_1", "deviceId": "dev_1", "bucket": "b", "key": "k"})}]}, None)
assert len(calls) == 1 and calls[0]["recording_id"] == "rec_1", calls
print("10 worker -> ingest.run called")

calls.clear()
worker.handler({"Records": [{"body": json.dumps(
    {"recordingId": "rec_2", "deviceId": None, "bucket": "b", "key": "k"})}]}, None)
assert calls == []
print("11 worker, no device -> skipped")

# rotation without a cold start: change the secret, sign with the new one
sent.clear()
SECRET = "rotated-token"
new_sig = hmac.new(SECRET.encode(), json.dumps(IMPORTED).encode(), hashlib.sha256).hexdigest()
r = receiver.handler(evt(IMPORTED, sig=new_sig), None)
assert r["statusCode"] == 200 and len(sent) == 1, (r, sent)
print("12 rotated token accepted without a cold start")

print("\nALL TESTS PASSED")
