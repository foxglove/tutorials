// Run the release data-loader component outside Foxglove.
//
// Requires wasm-tools and @bytecodealliance/jco on PATH (or WASM_TOOLS / JCO).
// DICOM_DATA_DIR should contain lidc/ and 4dlung/. Previews go to
// /tmp/dicom_plan/previews by default (PREVIEW_DIR).

import { execFileSync } from "node:child_process";
import fs from "node:fs";
import path from "node:path";
import zlib from "node:zlib";
import { pathToFileURL } from "node:url";

const tutorialRoot = path.resolve(import.meta.dirname, "..");
const dataRoot = process.env.DICOM_DATA_DIR ?? "/tmp/dicom_data";
const previewDir = process.env.PREVIEW_DIR ?? "/tmp/dicom_plan/previews";
const wasmTools = process.env.WASM_TOOLS ?? "wasm-tools";
const jco = process.env.JCO ?? "jco";
const coreWasm = path.join(
  tutorialRoot,
  "rust/target/wasm32-unknown-unknown/release/foxglove_dicom_data_loader.wasm",
);
const outDir = path.join(tutorialRoot, "scripts/.smoke");

function collectDcm(dir) {
  const out = [];
  const stack = [dir];
  while (stack.length > 0) {
    const current = stack.pop();
    for (const entry of fs.readdirSync(current, { withFileTypes: true })) {
      const full = path.join(current, entry.name);
      if (entry.isDirectory()) {
        stack.push(full);
      } else if (entry.name.toLowerCase().endsWith(".dcm")) {
        out.push(full);
      }
    }
  }
  out.sort();
  return out;
}

class Reader {
  constructor(filePath, memory) {
    this.data = fs.readFileSync(filePath);
    this.pos = 0;
    this.memory = memory;
  }

  read(ptr, len) {
    const n = Math.min(len, this.data.length - this.pos);
    if (n > 0) {
      new Uint8Array(this.memory().buffer).set(this.data.subarray(this.pos, this.pos + n), ptr);
      this.pos += n;
    }
    return BigInt(n);
  }
}

function decodeRawImage(buf) {
  const bytes = Buffer.from(buf);
  let offset = 0;
  let width;
  let height;
  let step;
  let encoding;
  let frameId;
  let data;
  const readVarint = () => {
    let value = 0;
    let shift = 0;
    while (offset < bytes.length) {
      const byte = bytes[offset];
      offset += 1;
      value |= (byte & 0x7f) << shift;
      if ((byte & 0x80) === 0) {
        return value;
      }
      shift += 7;
    }
    throw new Error("truncated varint");
  };
  while (offset < bytes.length) {
    const key = readVarint();
    const field = key >>> 3;
    const wire = key & 7;
    if (wire === 0) {
      readVarint();
    } else if (wire === 1) {
      offset += 8;
    } else if (wire === 5) {
      const value = bytes.readUInt32LE(offset);
      offset += 4;
      if (field === 2) {
        width = value;
      } else if (field === 3) {
        height = value;
      } else if (field === 5) {
        step = value;
      }
    } else if (wire === 2) {
      const len = readVarint();
      const slice = bytes.subarray(offset, offset + len);
      offset += len;
      if (field === 4) {
        encoding = slice.toString("utf8");
      } else if (field === 6) {
        data = slice;
      } else if (field === 7) {
        frameId = slice.toString("utf8");
      }
    } else {
      throw new Error(`unsupported protobuf wire type ${wire}`);
    }
  }
  if (width == null || height == null || data == null) {
    throw new Error("RawImage is missing width, height, or pixels");
  }
  return { width, height, step, encoding, frameId, data };
}

function crc32(buffer) {
  let crc = ~0;
  for (const byte of buffer) {
    crc ^= byte;
    for (let bit = 0; bit < 8; bit += 1) {
      crc = (crc >>> 1) ^ (0xedb88320 & -(crc & 1));
    }
  }
  return ~crc >>> 0;
}

function chunk(type, data) {
  const body = Buffer.concat([Buffer.from(type), data]);
  const out = Buffer.alloc(12 + data.length);
  out.writeUInt32BE(data.length, 0);
  body.copy(out, 4);
  out.writeUInt32BE(crc32(body), 8 + data.length);
  return out;
}

function writePng(file, image) {
  const { width, height, data } = image;
  if (data.length !== width * height) {
    throw new Error(`${file}: expected ${width * height} pixels, got ${data.length}`);
  }
  const raw = Buffer.alloc((width + 1) * height);
  for (let y = 0; y < height; y += 1) {
    data.copy(raw, y * (width + 1) + 1, y * width, (y + 1) * width);
  }
  const ihdr = Buffer.alloc(13);
  ihdr.writeUInt32BE(width, 0);
  ihdr.writeUInt32BE(height, 4);
  ihdr[8] = 8;
  ihdr[9] = 0;
  const png = Buffer.concat([
    Buffer.from([0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a]),
    chunk("IHDR", ihdr),
    chunk("IDAT", zlib.deflateSync(raw)),
    chunk("IEND", Buffer.alloc(0)),
  ]);
  fs.writeFileSync(file, png);
}

function transpile() {
  fs.rmSync(outDir, { recursive: true, force: true });
  fs.mkdirSync(outDir, { recursive: true });
  const component = path.join(outDir, "component.wasm");
  execFileSync(wasmTools, ["component", "new", coreWasm, "-o", component], { stdio: "inherit" });
  execFileSync(
    jco,
    [
      "transpile",
      component,
      "-o",
      outDir,
      "--name",
      "dicom",
      "--instantiation",
      "sync",
      "--no-nodejs-compat",
      "--no-typescript",
    ],
    { stdio: "inherit" },
  );
}

async function loadComponent() {
  const js = pathToFileURL(path.join(outDir, "dicom.js")).href;
  const imported = await import(js);
  return imported.instantiate;
}

async function play(name, dir, instantiate) {
  const files = collectDcm(dir);
  if (files.length === 0) {
    throw new Error(`no .dcm files under ${dir}`);
  }
  let memory;
  const memoryRef = () => {
    if (!memory) {
      throw new Error("guest memory is not available");
    }
    return memory;
  };
  let instance = instantiate(
    (filename) => new WebAssembly.Module(fs.readFileSync(path.join(outDir, filename))),
    {
      "foxglove:loader/console": {
        log(message) {
          console.log(`[${name}] ${message}`);
        },
      },
      "foxglove:loader/reader": {
        Reader,
        open(filePath) {
          return new Reader(filePath, memoryRef);
        },
      },
    },
    (module, importObject) => {
      const created = new WebAssembly.Instance(module, importObject);
      const exported = created.exports.memory;
      if (exported && (!memory || exported.buffer.byteLength >= memory.buffer.byteLength)) {
        memory = exported;
      }
      return created;
    },
  );
  if (instance instanceof Promise) {
    instance = await instance;
  }
  const loader = new instance.loader.DataLoader({ paths: files });
  const init = loader.initialize();
  const memoryBytes = memory.buffer.byteLength;
  console.log(`[${name}] wasm memory after initialize: ${memoryBytes} bytes (${files.length} files)`);
  const errors = init.problems.filter((problem) => problem.severity === "error");
  if (errors.length > 0) {
    throw new Error(`${name} initialize errors: ${errors.map((problem) => problem.message).join("; ")}`);
  }
  if (init.problems.length > 0) {
    throw new Error(`${name} unexpected problems: ${JSON.stringify(init.problems)}`);
  }
  const { startTime, endTime } = init.timeRange;
  if (startTime <= 0n || endTime < startTime) {
    throw new Error(`${name} time range ${startTime}..${endTime}`);
  }
  const ids = init.channels.map((channel) => channel.id);
  const iterator = loader.createIterator({ channels: Uint16Array.from(ids) });
  const counts = new Map();
  const images = new Map();
  let previous;
  let total = 0;
  while (true) {
    const next = iterator.next();
    if (next == null) {
      break;
    }
    if (next.tag !== "ok") {
      throw new Error(`${name} iterator error: ${next.val}`);
    }
    const message = next.val;
    if (previous != null && message.logTime < previous) {
      throw new Error(`${name} log time went backwards`);
    }
    if (message.logTime < startTime || message.logTime > endTime) {
      throw new Error(`${name} log time ${message.logTime} outside ${startTime}..${endTime}`);
    }
    previous = message.logTime;
    counts.set(message.channelId, (counts.get(message.channelId) ?? 0) + 1);
    const topic = init.channels.find((channel) => channel.id === message.channelId)?.topicName;
    if (topic === "/dicom/axial" || topic === "/dicom/coronal" || topic === "/dicom/sagittal") {
      const list = images.get(topic) ?? [];
      list.push(message.data);
      images.set(topic, list);
    }
    total += 1;
  }
  if (previous !== endTime) {
    throw new Error(`${name} last log time ${previous} != end ${endTime}`);
  }
  let expected = 0n;
  for (const channel of init.channels) {
    const got = BigInt(counts.get(channel.id) ?? 0);
    if (got !== channel.messageCount) {
      throw new Error(`${name} ${channel.topicName}: ${got} messages, expected ${channel.messageCount}`);
    }
    expected += channel.messageCount ?? 0n;
  }
  if (BigInt(total) !== expected) {
    throw new Error(`${name} iterated ${total}, expected ${expected}`);
  }
  const backfill = loader.getBackfill({ time: startTime, channels: Uint16Array.from(ids) });
  if (backfill.length !== ids.length) {
    throw new Error(`${name} backfill returned ${backfill.length} messages for ${ids.length} channels`);
  }
  return { images, memoryBytes, channels: init.channels };
}

function expectImage(images, topic, index, label) {
  const list = images.get(topic);
  if (!list || !list[index]) {
    throw new Error(`missing ${topic} frame ${index}`);
  }
  const image = decodeRawImage(list[index]);
  if (image.encoding !== "mono8") {
    throw new Error(`${label} encoding ${image.encoding}`);
  }
  if (image.width !== 512 || image.data.length !== image.width * image.height) {
    throw new Error(`${label} is ${image.width}x${image.height} (${image.data.length} bytes)`);
  }
  writePng(path.join(previewDir, `${label}.png`), image);
  console.log(`[preview] ${label}.png ${image.width}x${image.height} frame=${image.frameId}`);
  return image.data;
}

transpile();
const instantiate = await loadComponent();
fs.mkdirSync(previewDir, { recursive: true });

const chest = await play("lidc", path.join(dataRoot, "lidc"), instantiate);
expectImage(chest.images, "/dicom/axial", 66, "lidc-axial");
expectImage(chest.images, "/dicom/coronal", 0, "lidc-coronal");
expectImage(chest.images, "/dicom/sagittal", 0, "lidc-sagittal");
if (chest.images.get("/dicom/axial")?.length !== 133) {
  throw new Error(`lidc axial frames ${chest.images.get("/dicom/axial")?.length}`);
}

const breathing = await play("4dlung", path.join(dataRoot, "4dlung"), instantiate);
const phase0 = expectImage(breathing.images, "/dicom/coronal", 0, "4dlung-coronal-0");
const phase50 = expectImage(breathing.images, "/dicom/coronal", 5, "4dlung-coronal-50");
const phase50Cycle2 = breathing.images.get("/dicom/coronal")?.[15];
if (!phase50Cycle2) {
  throw new Error("missing 4D coronal frame 15");
}
if (Buffer.compare(phase0, phase50) === 0) {
  throw new Error("4D coronal phase 0% and 50% are identical");
}
if (Buffer.compare(phase50, decodeRawImage(phase50Cycle2).data) !== 0) {
  throw new Error("4D coronal phase 50% was not reused on the next cycle");
}
expectImage(breathing.images, "/dicom/axial", 0, "4dlung-axial");
expectImage(breathing.images, "/dicom/sagittal", 0, "4dlung-sagittal");
if (breathing.images.get("/dicom/coronal")?.length !== 30) {
  throw new Error(`4dlung coronal frames ${breathing.images.get("/dicom/coronal")?.length}`);
}

console.log(
  JSON.stringify(
    {
      lidcMemoryBytes: chest.memoryBytes,
      lung4dMemoryBytes: breathing.memoryBytes,
      previewDir,
    },
    null,
    2,
  ),
);
