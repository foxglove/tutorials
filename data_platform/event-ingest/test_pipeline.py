"""Offline tests for the whole pipeline. No AWS, no network.

    python3 test_pipeline.py
"""
import base64, hashlib, hmac, importlib.util, json, os, sys, types
from datetime import datetime, timezone

HERE = os.path.dirname(os.path.abspath(__file__))
MCAP = os.path.join(HERE, "sample_events.mcap")
sys.path.insert(0, HERE)

# Generated rather than committed: a fixed sample file is a duplicate-upload trap.
# Pinned base time so the timestamp assertions below are deterministic.
BASE = 1760000000
import write_events
write_events.main(MCAP, base=BASE)


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


# ---------------------------------------------------------------- fakes ----
class Resp:
    def __init__(self, p=None, content=b""): self._p, self.content = p, content
    def raise_for_status(self): pass
    def json(self): return self._p


class FakeAPI:
    """Routes the calls ingest.run makes. The signed link must be fetched without
    the API key, and only the event topics should be requested from the stream."""
    LINK = "https://signed.example/stream.mcap?sig=abc"

    def __init__(self):
        self.topics = [{"topic": "/events", "schemaName": "foxglove.Event",
                        "schemaEncoding": "protobuf"},
                       {"topic": "/json_events", "schemaName": "foxglove.Event",
                        "schemaEncoding": "jsonschema"},
                       {"topic": "/camera", "schemaName": "foxglove.RawImage"}]
        self.existing, self.posted, self.streamed = [], [], []

    def get(self, url, headers=None, **kw):
        if url == self.LINK:
            assert not headers, "signed link fetched with the API key attached"
            with open(MCAP, "rb") as f:
                return Resp(content=f.read())
        if url.endswith("/data/topics"):
            return Resp(self.topics)
        if url.endswith("/events"):
            return Resp(self.existing)
        raise AssertionError(f"unexpected GET {url}")

    def post(self, url, json=None, **kw):
        if url.endswith("/data/stream"):
            self.streamed.append(json["topics"])
            return Resp({"link": self.LINK})
        if url.endswith("/events"):
            self.posted.append(json)
            return Resp({"id": f"evt_{len(self.posted)}"})
        raise AssertionError(f"unexpected POST {url}")


# ------------------------------------------------------------- ingest -----
import ingest  # noqa: E402

api = FakeAPI()
ingest.requests = api
posted, existing = api.posted, api.existing

n = ingest.run("rec_1", "dev_1", "key")
assert n == 3, n
assert api.streamed == [["/events"]], api.streamed
expected_start = datetime.fromtimestamp(BASE, timezone.utc).isoformat().replace("+00:00", "Z")
assert posted[0]["start"] == expected_start, posted[0]
assert posted[0]["metadata"]["name"] == "pick_attempt"
assert posted[0]["metadata"]["source_recording_id"] == "rec_1"
print(f"1 ingest.run -> 3 events, streamed only {api.streamed[0]} (JSON event topic skipped)")

keys = [p["metadata"]["source_event_key"] for p in posted]
assert len(set(keys)) == 3 and all(k.startswith("/events:") for k in keys), keys
existing.extend({"id": f"evt_{i}", "metadata": p["metadata"]} for i, p in enumerate(posted))
before = len(posted)
assert ingest.run("rec_1", "dev_1", "key") == 0
assert len(posted) == before
print("2 second run -> deduped, nothing created")

# Partial failure: only event 0 landed before the worker died. The retry must
# fill in 1 and 2, not treat the recording as done.
del existing[1:]
posted.clear()
assert ingest.run("rec_1", "dev_1", "key") == 2
assert [p["metadata"]["source_event_key"] for p in posted] == keys[1:]
print("3 retry after partial failure -> missing events created")

api.topics = [{"topic": "/camera", "schemaName": "foxglove.RawImage"}]
api.streamed.clear()
assert ingest.run("rec_1", "dev_1", "key") == 0
assert api.streamed == []
print("4 no foxglove.Event topic -> nothing streamed")

# Keys must not depend on the order the stream returns messages in. Two events
# share a log time here; reversing the input must not change which key each gets.
def msg(topic, t, data, label):
    return (None, types.SimpleNamespace(topic=topic),
            types.SimpleNamespace(log_time=t, data=data), label)
batch = [msg("/events", 5, b"b", "B"), msg("/events", 5, b"a", "A"),
         msg("/events", 9, b"c", "C"), msg("/robot/events", 5, b"d", "D")]
forward = dict(ingest.keyed_events(batch))
backward = dict(ingest.keyed_events(list(reversed(batch))))
assert forward == backward, (forward, backward)
assert forward == {"/events:5:0": "A", "/events:5:1": "B",
                   "/events:9:0": "C", "/robot/events:5:0": "D"}, forward
print("5 event keys stable when the stream reorders same-timestamp events")

existing.append({"id": "evt_null", "metadata": None})
assert ingest.run("rec_1", "dev_1", "key") == 0
existing.pop()
print("6 existing event with null metadata -> no crash")

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
    "id": "rec_1", "device": {"id": "dev_1"}}}

cases = [
    ("valid signature",      evt(IMPORTED),                                200, 1),
    ("base64 body",          evt(IMPORTED, sig=sign(json.dumps(IMPORTED).encode()), b64=True), 200, 1),
    ("bad signature",        evt(IMPORTED, sig="deadbeef"),                401, 0),
    ("no signature header",  {**evt(IMPORTED), "headers": {}},             401, 0),
    ("other event type",     evt({"type": "ping"}),                        200, 0),
    ("no device",            evt({"type": "recording.imported",
                                  "recording": {"id": "rec_2"}}),          200, 0),
]
for i, (name, e, code, enqueued) in enumerate(cases, start=7):
    sent.clear()
    r = receiver.handler(e, None)
    assert r["statusCode"] == code, (name, r)
    assert len(sent) == enqueued, (name, sent)
    print(f"{i} receiver, {name} -> {code}, {len(sent)} enqueued")

# tampered body with the original signature
sent.clear()
tampered = json.loads(json.dumps(IMPORTED)); tampered["recording"]["device"]["id"] = "evil"
assert receiver.handler(evt(tampered, sig=sign(json.dumps(IMPORTED).encode())), None)["statusCode"] == 401
assert sent == []
print("13 receiver, tampered body -> 401, nothing enqueued")

worker = load("worker_handler", f"{HERE}/lambda/worker/handler.py")
calls = []
worker.ingest = types.SimpleNamespace(run=lambda **kw: calls.append(kw))

worker.handler({"Records": [{"body": json.dumps(
    {"recordingId": "rec_1", "deviceId": "dev_1"})}]}, None)
assert calls == [{"recording_id": "rec_1", "device_id": "dev_1", "api_key": "fox_sk"}], calls
print("14 worker -> ingest.run called")

calls.clear()
worker.handler({"Records": [{"body": json.dumps(
    {"recordingId": "rec_2", "deviceId": None})}]}, None)
assert calls == []
print("15 worker, no device (queued by an older receiver) -> skipped")

# rotation without a cold start: change the secret, sign with the new one
sent.clear()
SECRET = "rotated-token"
new_sig = hmac.new(SECRET.encode(), json.dumps(IMPORTED).encode(), hashlib.sha256).hexdigest()
r = receiver.handler(evt(IMPORTED, sig=new_sig), None)
assert r["statusCode"] == 200 and len(sent) == 1, (r, sent)
print("16 rotated token accepted without a cold start")

print("\nALL TESTS PASSED")
