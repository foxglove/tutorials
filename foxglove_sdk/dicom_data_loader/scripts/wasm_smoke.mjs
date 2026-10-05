// Play the release loader outside Foxglove.
// Requires wasm-tools and jco (WASM_TOOLS, JCO). Data is data/chest and data/breathing
// unless DICOM_DATA_DIR is set. PREVIEW_DIR, if set, receives grayscale PNGs.

import { execFileSync } from "node:child_process";
import fs from "node:fs";
import path from "node:path";
import zlib from "node:zlib";
import { pathToFileURL } from "node:url";
const root = path.resolve(import.meta.dirname, "..");
const dataRoot = process.env.DICOM_DATA_DIR ?? path.join(root, "data");
const previewDir = process.env.PREVIEW_DIR;
const wasmTools = process.env.WASM_TOOLS ?? "wasm-tools";
const jco = process.env.JCO ?? "jco";
const coreWasm = path.join(
  root,
  "rust/target/wasm32-unknown-unknown/release/foxglove_dicom_data_loader.wasm",
);
const outDir = path.join(root, "scripts/.smoke");

function collectDcm(dir) {
  const out = [];
  const stack = [dir];
  while (stack.length > 0) {
    const current = stack.pop();
    for (const entry of fs.readdirSync(current, { withFileTypes: true })) {
      const full = path.join(current, entry.name);
      if (entry.isDirectory()) stack.push(full);
      else if (entry.name.toLowerCase().endsWith(".dcm")) out.push(full);
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

  // Absolute position. The guest maps SeekFrom::* onto this plus position() and size().
  seek(pos) {
    const requested = BigInt(pos);
    const len = BigInt(this.data.length);
    const at = requested < 0n ? 0n : requested > len ? len : requested;
    this.pos = Number(at);
    return at;
  }

  position() {
    return BigInt(this.pos);
  }

  read(ptr, len) {
    const n = Math.min(len, this.data.length - this.pos);
    if (n > 0) {
      new Uint8Array(this.memory().buffer).set(this.data.subarray(this.pos, this.pos + n), ptr);
      this.pos += n;
    }
    return BigInt(n);
  }

  size() {
    return BigInt(this.data.length);
  }
}

function rawImage(buf) {
  const bytes = Buffer.from(buf);
  let offset = 0;
  let width;
  let height;
  let data;
  const varint = () => {
    let value = 0;
    let shift = 0;
    for (;;) {
      const byte = bytes[offset++];
      value |= (byte & 0x7f) << shift;
      if ((byte & 0x80) === 0) return value;
      shift += 7;
    }
  };
  while (offset < bytes.length) {
    const key = varint();
    const wire = key & 7;
    const field = key >>> 3;
    if (wire === 0) varint();
    else if (wire === 1) offset += 8;
    else if (wire === 5) {
      if (field === 2) width = bytes.readUInt32LE(offset);
      if (field === 3) height = bytes.readUInt32LE(offset);
      offset += 4;
    } else if (wire === 2) {
      const len = varint();
      if (field === 6) data = bytes.subarray(offset, offset + len);
      offset += len;
    } else throw new Error(`wire ${wire}`);
  }
  if (width == null || height == null || !data) throw new Error("RawImage is incomplete");
  return { width, height, data };
}

function writePng(file, { width, height, data }) {
  const raw = Buffer.alloc((width + 1) * height);
  for (let y = 0; y < height; y += 1)
    data.copy(raw, y * (width + 1) + 1, y * width, (y + 1) * width);
  const chunk = (type, payload) => {
    const body = Buffer.concat([Buffer.from(type), payload]);
    let crc = ~0;
    for (const byte of body) {
      crc ^= byte;
      for (let bit = 0; bit < 8; bit += 1) crc = (crc >>> 1) ^ (0xedb88320 & -(crc & 1));
    }
    const out = Buffer.alloc(12 + payload.length);
    out.writeUInt32BE(payload.length, 0);
    body.copy(out, 4);
    out.writeUInt32BE(~crc >>> 0, 8 + payload.length);
    return out;
  };
  const ihdr = Buffer.alloc(13);
  ihdr.writeUInt32BE(width, 0);
  ihdr.writeUInt32BE(height, 4);
  ihdr[8] = 8;
  const sig = Buffer.from([0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a]);
  fs.writeFileSync(
    file,
    Buffer.concat([
      sig,
      chunk("IHDR", ihdr),
      chunk("IDAT", zlib.deflateSync(raw)),
      chunk("IEND", Buffer.alloc(0)),
    ]),
  );
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

async function play(name, dir, instantiate) {
  const files = collectDcm(dir);
  if (files.length === 0) throw new Error(`no .dcm files under ${dir}`);
  let memory;
  const memoryRef = () => {
    if (!memory) throw new Error("guest memory is not available");
    return memory;
  };
  let instance = instantiate(
    (filename) => new WebAssembly.Module(fs.readFileSync(path.join(outDir, filename))),
    {
      "foxglove:loader/console": { log: (message) => console.log(`[${name}] ${message}`) },
      "foxglove:loader/reader": { Reader, open: (filePath) => new Reader(filePath, memoryRef) },
    },
    (module, importObject) => {
      const created = new WebAssembly.Instance(module, importObject);
      const exported = created.exports.memory;
      if (exported && (!memory || exported.buffer.byteLength >= memory.buffer.byteLength))
        memory = exported;
      return created;
    },
  );
  if (instance instanceof Promise) instance = await instance;
  const loader = new instance.loader.DataLoader({ paths: files });
  const init = loader.initialize();
  const memoryBytes = memory.buffer.byteLength;
  console.log(
    `[${name}] wasm memory after initialize: ${memoryBytes} bytes (${files.length} files)`,
  );
  if (init.problems.length > 0) throw new Error(`${name}: ${JSON.stringify(init.problems)}`);
  const { startTime, endTime } = init.timeRange;
  if (!(startTime > 0n && endTime >= startTime))
    throw new Error(`${name} time range ${startTime}..${endTime}`);
  const ids = init.channels.map((channel) => channel.id);
  const iterator = loader.createIterator({ channels: Uint16Array.from(ids) });
  const counts = new Map();
  const images = new Map();
  let previous;
  let total = 0;
  while (true) {
    const next = iterator.next();
    if (next == null) break;
    if (next.tag !== "ok") throw new Error(`${name} iterator error: ${next.val}`);
    const message = next.val;
    if (previous != null && message.logTime < previous)
      throw new Error(`${name} log time went backwards`);
    if (message.logTime < startTime || message.logTime > endTime)
      throw new Error(`${name} log time out of range`);
    previous = message.logTime;
    counts.set(message.channelId, (counts.get(message.channelId) ?? 0) + 1);
    if (previewDir) {
      const topic = init.channels.find((channel) => channel.id === message.channelId)?.topicName;
      if (topic === "/dicom/axial" || topic === "/dicom/coronal" || topic === "/dicom/sagittal") {
        const list = images.get(topic) ?? [];
        list.push(message.data);
        images.set(topic, list);
      }
    }
    total += 1;
  }
  if (previous !== endTime) throw new Error(`${name} last log time ${previous} != end ${endTime}`);
  let expected = 0n;
  for (const channel of init.channels) {
    const got = BigInt(counts.get(channel.id) ?? 0);
    if (got !== channel.messageCount)
      throw new Error(`${name} ${channel.topicName}: ${got} != ${channel.messageCount}`);
    expected += channel.messageCount ?? 0n;
  }
  if (BigInt(total) !== expected)
    throw new Error(`${name} iterated ${total}, expected ${expected}`);
  const backfill = loader.getBackfill({ time: startTime, channels: Uint16Array.from(ids) });
  if (backfill.length !== ids.length)
    throw new Error(`${name} backfill ${backfill.length} != ${ids.length}`);
  console.log(`[${name}] wasm memory after playback: ${memory.buffer.byteLength} bytes`);
  return { images, memoryBytes: Math.max(memoryBytes, memory.buffer.byteLength) };
}

function save(images, topic, index, label) {
  const image = rawImage(images.get(topic)[index]);
  writePng(path.join(previewDir, `${label}.png`), image);
  console.log(`[preview] ${label}.png ${image.width}x${image.height}`);
  return image.data;
}

transpile();
const imported = await import(pathToFileURL(path.join(outDir, "dicom.js")).href);
if (previewDir) fs.mkdirSync(previewDir, { recursive: true });
const chest = await play("chest", path.join(dataRoot, "chest"), imported.instantiate);
const breathing = await play("breathing", path.join(dataRoot, "breathing"), imported.instantiate);
if (previewDir) {
  save(chest.images, "/dicom/axial", 66, "lidc-axial");
  save(chest.images, "/dicom/coronal", 0, "lidc-coronal");
  save(chest.images, "/dicom/sagittal", 0, "lidc-sagittal");
  const phase0 = save(breathing.images, "/dicom/coronal", 0, "4dlung-coronal-0");
  const phase50 = save(breathing.images, "/dicom/coronal", 5, "4dlung-coronal-50");
  save(breathing.images, "/dicom/axial", 0, "4dlung-axial");
  save(breathing.images, "/dicom/sagittal", 0, "4dlung-sagittal");
  if (Buffer.compare(phase0, phase50) === 0)
    throw new Error("breathing coronal phases are identical");
}
console.log(
  JSON.stringify(
    { chestMemoryBytes: chest.memoryBytes, breathingMemoryBytes: breathing.memoryBytes },
    null,
    2,
  ),
);
