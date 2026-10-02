export type Vec3 = readonly [number, number, number];
export type Quat = readonly [number, number, number, number];

export type PoseSample = {
  position: Vec3;
  orientation: Quat;
  frameId: string | undefined;
};

export type ExtractPoseOptions = {
  childFrameId?: string;
  childFrameLock?: ChildFrameLock;
};

export class ChildFrameLock {
  #locked: string | undefined;
  #seen: string[] = [];

  reset(): void {
    this.#locked = undefined;
    this.#seen = [];
  }

  observed(): readonly string[] {
    return this.#seen;
  }

  accept(actual: string | undefined, wanted: string | undefined): boolean {
    if (actual != undefined && actual.length > 0 && !this.#seen.includes(actual)) {
      this.#seen.push(actual);
    }
    if (wanted != undefined && wanted.length > 0) {
      return actual === wanted;
    }
    if (this.#locked == undefined) {
      if (actual == undefined || actual.length === 0) {
        return true;
      }
      this.#locked = actual;
      return true;
    }
    return actual === this.#locked;
  }
}

const POSE_IN_FRAME = "foxglove.PoseInFrame";
const POSES_IN_FRAME = "foxglove.PosesInFrame";
const FRAME_TRANSFORM = "foxglove.FrameTransform";
const FRAME_TRANSFORMS = "foxglove.FrameTransforms";

const POSE_STAMPED = new Set<string>([
  "geometry_msgs/PoseStamped",
  "geometry_msgs/msg/PoseStamped",
]);

const ODOMETRY = new Set<string>(["nav_msgs/Odometry", "nav_msgs/msg/Odometry"]);

const TF_MESSAGE = new Set<string>(["tf2_msgs/TFMessage", "tf2_msgs/msg/TFMessage"]);

const TRANSFORM_STAMPED = new Set<string>([
  "geometry_msgs/TransformStamped",
  "geometry_msgs/msg/TransformStamped",
]);

const CHILD_FRAME_SCHEMAS = new Set<string>([
  FRAME_TRANSFORM,
  FRAME_TRANSFORMS,
  ...TF_MESSAGE,
  ...TRANSFORM_STAMPED,
]);

export const SUPPORTED_SCHEMAS: readonly string[] = [
  POSE_IN_FRAME,
  POSES_IN_FRAME,
  FRAME_TRANSFORM,
  FRAME_TRANSFORMS,
  ...POSE_STAMPED,
  ...ODOMETRY,
  ...TF_MESSAGE,
  ...TRANSFORM_STAMPED,
];

export function isSupportedSchema(schemaName: string): boolean {
  return SUPPORTED_SCHEMAS.includes(schemaName);
}

export function schemaUsesChildFrame(schemaName: string): boolean {
  return CHILD_FRAME_SCHEMAS.has(schemaName);
}

export function extractPose(
  schemaName: string,
  message: unknown,
  options?: ExtractPoseOptions,
): PoseSample | undefined {
  const record = asRecord(message);
  if (!record) {
    return undefined;
  }
  const childFrameId = options?.childFrameId;
  const childFrameLock = options?.childFrameLock;
  switch (schemaName) {
    case POSE_IN_FRAME:
      return poseSample(parsePose(record["pose"]), readString(record["frame_id"]));
    case POSES_IN_FRAME: {
      const poses = asArray(record["poses"]);
      const first = poses?.[0];
      return poseSample(parsePose(first), readString(record["frame_id"]));
    }
    case FRAME_TRANSFORM:
      return fromFoxgloveTransform(record, childFrameId, childFrameLock);
    case FRAME_TRANSFORMS:
      return firstMatching(asArray(record["transforms"]), childFrameId, childFrameLock, fromFoxgloveTransform);
    default:
      break;
  }
  if (POSE_STAMPED.has(schemaName)) {
    return poseSample(parsePose(record["pose"]), headerFrameId(record["header"]));
  }
  if (ODOMETRY.has(schemaName)) {
    const poseWithCov = asRecord(record["pose"]);
    return poseSample(parsePose(poseWithCov?.["pose"]), headerFrameId(record["header"]));
  }
  if (TF_MESSAGE.has(schemaName)) {
    return firstMatching(asArray(record["transforms"]), childFrameId, childFrameLock, fromRosTransform);
  }
  if (TRANSFORM_STAMPED.has(schemaName)) {
    return fromRosTransform(record, childFrameId, childFrameLock);
  }
  return undefined;
}

function poseSample(
  pose: { position: Vec3; orientation: Quat } | undefined,
  frameId: string | undefined,
): PoseSample | undefined {
  if (!pose) {
    return undefined;
  }
  return { position: pose.position, orientation: pose.orientation, frameId };
}

function fromFoxgloveTransform(
  message: unknown,
  childFrameId: string | undefined,
  childFrameLock: ChildFrameLock | undefined,
): PoseSample | undefined {
  const record = asRecord(message);
  if (!record) {
    return undefined;
  }
  const child = readString(record["child_frame_id"]);
  if (!childFrameMatches(child, childFrameId, childFrameLock)) {
    return undefined;
  }
  const position = parseVec3(record["translation"]);
  const orientation = parseQuat(record["rotation"]);
  if (!position || !orientation) {
    return undefined;
  }
  return { position, orientation, frameId: readString(record["parent_frame_id"]) };
}

function fromRosTransform(
  message: unknown,
  childFrameId: string | undefined,
  childFrameLock: ChildFrameLock | undefined,
): PoseSample | undefined {
  const record = asRecord(message);
  if (!record) {
    return undefined;
  }
  const child = readString(record["child_frame_id"]);
  if (!childFrameMatches(child, childFrameId, childFrameLock)) {
    return undefined;
  }
  const transform = asRecord(record["transform"]);
  if (!transform) {
    return undefined;
  }
  const position = parseVec3(transform["translation"]);
  const orientation = parseQuat(transform["rotation"]);
  if (!position || !orientation) {
    return undefined;
  }
  return { position, orientation, frameId: headerFrameId(record["header"]) };
}

function firstMatching(
  items: readonly unknown[] | undefined,
  childFrameId: string | undefined,
  childFrameLock: ChildFrameLock | undefined,
  read: (
    message: unknown,
    childFrameId: string | undefined,
    childFrameLock: ChildFrameLock | undefined,
  ) => PoseSample | undefined,
): PoseSample | undefined {
  if (!items) {
    return undefined;
  }
  for (const item of items) {
    const pose = read(item, childFrameId, childFrameLock);
    if (pose) {
      return pose;
    }
  }
  return undefined;
}

function childFrameMatches(
  actual: string | undefined,
  wanted: string | undefined,
  childFrameLock: ChildFrameLock | undefined,
): boolean {
  if (childFrameLock) {
    return childFrameLock.accept(actual, wanted);
  }
  if (wanted == undefined || wanted.length === 0) {
    return true;
  }
  return actual === wanted;
}

function headerFrameId(header: unknown): string | undefined {
  const record = asRecord(header);
  if (!record) {
    return undefined;
  }
  return readString(record["frame_id"]);
}

function parsePose(value: unknown): { position: Vec3; orientation: Quat } | undefined {
  const record = asRecord(value);
  if (!record) {
    return undefined;
  }
  const position = parseVec3(record["position"]);
  const orientation = parseQuat(record["orientation"]);
  if (!position || !orientation) {
    return undefined;
  }
  return { position, orientation };
}

function parseVec3(value: unknown): Vec3 | undefined {
  const record = asRecord(value);
  if (!record) {
    return undefined;
  }
  const x = finiteNumber(record["x"]);
  const y = finiteNumber(record["y"]);
  const z = finiteNumber(record["z"]);
  if (x == undefined || y == undefined || z == undefined) {
    return undefined;
  }
  return [x, y, z];
}

function parseQuat(value: unknown): Quat | undefined {
  const record = asRecord(value);
  if (!record) {
    return undefined;
  }
  const x = finiteNumber(record["x"]);
  const y = finiteNumber(record["y"]);
  const z = finiteNumber(record["z"]);
  const w = finiteNumber(record["w"]);
  if (x == undefined || y == undefined || z == undefined || w == undefined) {
    return undefined;
  }
  const length = Math.hypot(x, y, z, w);
  if (length < 1e-8) {
    return undefined;
  }
  return [x / length, y / length, z / length, w / length];
}

function finiteNumber(value: unknown): number | undefined {
  if (typeof value !== "number" || !Number.isFinite(value)) {
    return undefined;
  }
  return value;
}

function readString(value: unknown): string | undefined {
  if (typeof value !== "string") {
    return undefined;
  }
  return value;
}

function asRecord(value: unknown): Record<string, unknown> | undefined {
  if (typeof value !== "object" || value == null) {
    return undefined;
  }
  return value as Record<string, unknown>;
}

function asArray(value: unknown): readonly unknown[] | undefined {
  if (!Array.isArray(value)) {
    return undefined;
  }
  const items: unknown[] = [];
  for (const item of value as readonly unknown[]) {
    items.push(item);
  }
  return items;
}
