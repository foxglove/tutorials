import type { Quat, Vec3 } from "./extractPose";

export type InterpolationMode = "interpolate" | "previous";

export type ReceiveTime = {
  sec: number;
  nsec: number;
};

export type TimedPose = {
  tSec: number;
  receiveTime?: ReceiveTime;
  position: Vec3;
  orientation: Quat;
  frameId: string | undefined;
};

export class PoseTimeline {
  #samples: TimedPose[] = [];

  size(): number {
    return this.#samples.length;
  }

  clear(): void {
    this.#samples = [];
  }

  insert(sample: TimedPose): void {
    this.insertMany([sample]);
  }

  insertMany(incoming: readonly TimedPose[]): void {
    if (incoming.length === 0) {
      return;
    }
    const compact = compactSorted(incoming);
    if (this.#samples.length === 0) {
      this.#samples = compact;
      return;
    }
    this.#samples = mergeSamples(this.#samples, compact);
  }

  sample(tSec: number, mode: InterpolationMode): TimedPose | undefined {
    const count = this.#samples.length;
    if (count === 0) {
      return undefined;
    }
    const first = this.#samples[0];
    const last = this.#samples[count - 1];
    if (!first || !last) {
      return undefined;
    }
    if (tSec < first.tSec) {
      return undefined;
    }
    if (tSec >= last.tSec) {
      return last;
    }
    const index = lowerBound(this.#samples, tSec);
    const at = this.#samples[index];
    if (!at) {
      return last;
    }
    if (at.tSec === tSec || mode === "previous") {
      if (at.tSec === tSec) {
        return at;
      }
      return this.#samples[index - 1];
    }
    const previous = this.#samples[index - 1];
    if (!previous) {
      return undefined;
    }
    const span = at.tSec - previous.tSec;
    const alpha = span === 0 ? 0 : (tSec - previous.tSec) / span;
    return {
      tSec,
      position: lerpVec3(previous.position, at.position, alpha),
      orientation: slerpQuat(previous.orientation, at.orientation, alpha),
      frameId: previous.frameId,
    };
  }

  rebase(position: Vec3): Vec3 {
    const origin = this.#samples[0]?.position;
    if (!origin) {
      return position;
    }
    return [position[0] - origin[0], position[1] - origin[1], position[2] - origin[2]];
  }

  path(): Float32Array {
    const out = new Float32Array(this.#samples.length * 3);
    for (let index = 0; index < this.#samples.length; index += 1) {
      const sample = this.#samples[index];
      if (!sample) {
        continue;
      }
      const position = this.rebase(sample.position);
      out[index * 3] = position[0];
      out[index * 3 + 1] = position[1];
      out[index * 3 + 2] = position[2];
    }
    return out;
  }

  times(): Float64Array {
    const out = new Float64Array(this.#samples.length);
    for (let index = 0; index < this.#samples.length; index += 1) {
      const sample = this.#samples[index];
      if (sample) {
        out[index] = sample.tSec;
      }
    }
    return out;
  }

  receiveTimes(): ReceiveTime[] {
    const out: ReceiveTime[] = [];
    for (const sample of this.#samples) {
      out.push(sample.receiveTime ?? timeFromSec(sample.tSec));
    }
    return out;
  }

  pathSlice(tStart: number, tEnd: number, mode: InterpolationMode = "interpolate"): Float32Array {
    const from = Math.min(tStart, tEnd);
    const to = Math.max(tStart, tEnd);
    const startPose = this.sample(from, mode);
    const endPose = this.sample(to, mode);
    if (!startPose || !endPose) {
      return new Float32Array();
    }
    const points: number[] = [...this.rebase(startPose.position)];
    const firstInterior = firstAfter(this.#samples, from);
    const endExclusive = lowerBound(this.#samples, to);
    for (let index = firstInterior; index < endExclusive; index += 1) {
      const sample = this.#samples[index];
      if (!sample) {
        continue;
      }
      const position = this.rebase(sample.position);
      points.push(position[0], position[1], position[2]);
    }
    const end = this.rebase(endPose.position);
    const tail = points.length;
    const sameEnd =
      tail >= 3 &&
      points[tail - 3] === end[0] &&
      points[tail - 2] === end[1] &&
      points[tail - 1] === end[2];
    if (!sameEnd) {
      points.push(end[0], end[1], end[2]);
    }
    return Float32Array.from(points);
  }
}

function compactSorted(incoming: readonly TimedPose[]): TimedPose[] {
  const sorted = [...incoming].sort((a, b) => a.tSec - b.tSec);
  const compact: TimedPose[] = [];
  for (const sample of sorted) {
    const last = compact[compact.length - 1];
    if (last?.tSec === sample.tSec) {
      compact[compact.length - 1] = sample;
    } else {
      compact.push(sample);
    }
  }
  return compact;
}

function mergeSamples(current: readonly TimedPose[], incoming: readonly TimedPose[]): TimedPose[] {
  const merged: TimedPose[] = [];
  let left = 0;
  let right = 0;
  while (left < current.length && right < incoming.length) {
    const a = current[left];
    const b = incoming[right];
    if (!a || !b) {
      break;
    }
    if (a.tSec < b.tSec) {
      merged.push(a);
      left += 1;
    } else if (b.tSec < a.tSec) {
      merged.push(b);
      right += 1;
    } else {
      merged.push(b);
      left += 1;
      right += 1;
    }
  }
  while (left < current.length) {
    const sample = current[left];
    if (sample) {
      merged.push(sample);
    }
    left += 1;
  }
  while (right < incoming.length) {
    const sample = incoming[right];
    if (sample) {
      merged.push(sample);
    }
    right += 1;
  }
  return merged;
}

function firstAfter(samples: readonly TimedPose[], tSec: number): number {
  const index = lowerBound(samples, tSec);
  const sample = samples[index];
  if (sample?.tSec === tSec) {
    return index + 1;
  }
  return index;
}

function timeFromSec(tSec: number): ReceiveTime {
  const sec = Math.floor(tSec);
  const nsec = Math.round((tSec - sec) * 1e9);
  if (nsec >= 1e9) {
    return { sec: sec + 1, nsec: 0 };
  }
  if (nsec < 0) {
    return { sec: sec - 1, nsec: 1e9 + nsec };
  }
  return { sec, nsec };
}

function lowerBound(samples: readonly TimedPose[], tSec: number): number {
  let lo = 0;
  let hi = samples.length;
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    const sample = samples[mid];
    if (sample && sample.tSec < tSec) {
      lo = mid + 1;
    } else {
      hi = mid;
    }
  }
  return lo;
}

function lerpVec3(a: Vec3, b: Vec3, alpha: number): Vec3 {
  return [
    a[0] + (b[0] - a[0]) * alpha,
    a[1] + (b[1] - a[1]) * alpha,
    a[2] + (b[2] - a[2]) * alpha,
  ];
}

function slerpQuat(a: Quat, b: Quat, alpha: number): Quat {
  let bx = b[0];
  let by = b[1];
  let bz = b[2];
  let bw = b[3];
  let dot = a[0] * bx + a[1] * by + a[2] * bz + a[3] * bw;
  if (dot < 0) {
    dot = -dot;
    bx = -bx;
    by = -by;
    bz = -bz;
    bw = -bw;
  }
  dot = Math.min(1, dot);
  if (dot > 0.9995) {
    return normalizeQuat([
      a[0] + (bx - a[0]) * alpha,
      a[1] + (by - a[1]) * alpha,
      a[2] + (bz - a[2]) * alpha,
      a[3] + (bw - a[3]) * alpha,
    ]);
  }
  const theta0 = Math.acos(dot);
  const theta = theta0 * alpha;
  const sinTheta = Math.sin(theta);
  const sinTheta0 = Math.sin(theta0);
  const scale0 = Math.cos(theta) - (dot * sinTheta) / sinTheta0;
  const scale1 = sinTheta / sinTheta0;
  return normalizeQuat([
    scale0 * a[0] + scale1 * bx,
    scale0 * a[1] + scale1 * by,
    scale0 * a[2] + scale1 * bz,
    scale0 * a[3] + scale1 * bw,
  ]);
}

function normalizeQuat(q: readonly [number, number, number, number]): Quat {
  const length = Math.hypot(q[0], q[1], q[2], q[3]);
  if (length < 1e-12) {
    return [0, 0, 0, 1];
  }
  return [q[0] / length, q[1] / length, q[2] / length, q[3] / length];
}
