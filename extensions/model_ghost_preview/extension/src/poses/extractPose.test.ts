import { describe, expect, it } from "vitest";

import { ChildFrameLock, extractPose, isSupportedSchema, schemaUsesChildFrame } from "./extractPose";

const position = { x: 1, y: 2, z: 3 };
const orientation = { x: 0, y: 0, z: 0, w: 1 };
const pose = { position, orientation };

describe("extractPose", () => {
  it("reads foxglove.PoseInFrame", () => {
    const sample = extractPose("foxglove.PoseInFrame", {
      frame_id: "map",
      pose,
    });
    expect(sample).toEqual({
      position: [1, 2, 3],
      orientation: [0, 0, 0, 1],
      frameId: "map",
    });
  });

  it("uses the first pose of foxglove.PosesInFrame", () => {
    const sample = extractPose("foxglove.PosesInFrame", {
      frame_id: "map",
      poses: [pose, { position: { x: 9, y: 9, z: 9 }, orientation }],
    });
    expect(sample?.position).toEqual([1, 2, 3]);
  });

  it("filters foxglove frame transforms by child frame", () => {
    const transforms = {
      transforms: [
        {
          parent_frame_id: "map",
          child_frame_id: "base",
          translation: { x: 4, y: 0, z: 0 },
          rotation: orientation,
        },
        {
          parent_frame_id: "map",
          child_frame_id: "truck",
          translation: position,
          rotation: orientation,
        },
      ],
    };
    expect(extractPose("foxglove.FrameTransforms", transforms, { childFrameId: "truck" })?.position).toEqual([
      1, 2, 3,
    ]);
    expect(extractPose("foxglove.FrameTransforms", transforms, { childFrameId: "missing" })).toBeUndefined();
    expect(extractPose("foxglove.FrameTransforms", transforms)?.position).toEqual([4, 0, 0]);
    expect(
      extractPose("foxglove.FrameTransform", transforms.transforms[0], { childFrameId: "truck" }),
    ).toBeUndefined();
    expect(extractPose("foxglove.FrameTransform", transforms.transforms[1])?.frameId).toBe("map");
  });

  it("locks an empty child frame filter onto the first child id in the range", () => {
    const lock = new ChildFrameLock();
    const base = {
      parent_frame_id: "map",
      child_frame_id: "base",
      translation: { x: 4, y: 0, z: 0 },
      rotation: orientation,
    };
    const truck = {
      parent_frame_id: "map",
      child_frame_id: "truck",
      translation: position,
      rotation: orientation,
    };
    expect(
      extractPose("foxglove.FrameTransforms", { transforms: [base, truck] }, { childFrameLock: lock })
        ?.position,
    ).toEqual([4, 0, 0]);
    expect(
      extractPose("foxglove.FrameTransforms", { transforms: [truck, base] }, { childFrameLock: lock })
        ?.position,
    ).toEqual([4, 0, 0]);
    expect(lock.observed()).toEqual(["base", "truck"]);
    lock.reset();
    expect(lock.observed()).toEqual([]);
    expect(
      extractPose("foxglove.FrameTransforms", { transforms: [truck, base] }, { childFrameLock: lock })
        ?.position,
    ).toEqual([1, 2, 3]);
    expect(
      extractPose(
        "tf2_msgs/TFMessage",
        {
          transforms: [
            {
              header: { frame_id: "map" },
              child_frame_id: "base",
              transform: { translation: { x: 8, y: 0, z: 0 }, rotation: orientation },
            },
          ],
        },
        { childFrameLock: lock },
      ),
    ).toBeUndefined();
  });

  it("reads geometry_msgs PoseStamped, including the /msg form", () => {
    const message = { header: { frame_id: "odom" }, pose };
    expect(extractPose("geometry_msgs/PoseStamped", message)?.frameId).toBe("odom");
    expect(extractPose("geometry_msgs/msg/PoseStamped", message)?.position).toEqual([1, 2, 3]);
  });

  it("reads nav_msgs Odometry pose.pose", () => {
    const message = {
      header: { frame_id: "map" },
      child_frame_id: "base",
      pose: { pose },
    };
    expect(extractPose("nav_msgs/Odometry", message)?.position).toEqual([1, 2, 3]);
    expect(extractPose("nav_msgs/msg/Odometry", message)?.frameId).toBe("map");
  });

  it("filters tf2 TFMessage transforms by child_frame_id", () => {
    const message = {
      transforms: [
        {
          header: { frame_id: "map" },
          child_frame_id: "sensor",
          transform: { translation: { x: 8, y: 0, z: 0 }, rotation: orientation },
        },
        {
          header: { frame_id: "map" },
          child_frame_id: "truck",
          transform: { translation: position, rotation: orientation },
        },
      ],
    };
    expect(extractPose("tf2_msgs/TFMessage", message, { childFrameId: "truck" })?.position).toEqual([
      1, 2, 3,
    ]);
    expect(extractPose("tf2_msgs/msg/TFMessage", message, { childFrameId: "sensor" })?.position).toEqual([
      8, 0, 0,
    ]);
  });

  it("reads geometry_msgs TransformStamped", () => {
    const message = {
      header: { frame_id: "map" },
      child_frame_id: "truck",
      transform: { translation: position, rotation: orientation },
    };
    expect(extractPose("geometry_msgs/TransformStamped", message)?.position).toEqual([1, 2, 3]);
    expect(
      extractPose("geometry_msgs/msg/TransformStamped", message, { childFrameId: "other" }),
    ).toBeUndefined();
  });

  it("normalizes quaternions and rejects invalid messages", () => {
    const sample = extractPose("foxglove.PoseInFrame", {
      frame_id: "map",
      pose: { position, orientation: { x: 0, y: 0, z: 0, w: 2 } },
    });
    expect(sample?.orientation).toEqual([0, 0, 0, 1]);
    expect(extractPose("foxglove.PoseInFrame", null)).toBeUndefined();
    expect(extractPose("foxglove.PoseInFrame", { pose: { position } })).toBeUndefined();
    expect(extractPose("foxglove.PosesInFrame", { poses: [] })).toBeUndefined();
    expect(extractPose("foxglove.FrameTransforms", { transforms: "nope" })).toBeUndefined();
    expect(extractPose("sensor_msgs/Imu", pose)).toBeUndefined();
    expect(
      extractPose("foxglove.PoseInFrame", {
        pose: { position, orientation: { x: 0, y: 0, z: 0, w: 0 } },
      }),
    ).toBeUndefined();
  });
});

describe("schema helpers", () => {
  it("recognizes supported schemas and which ones need a child frame", () => {
    expect(isSupportedSchema("foxglove.PoseInFrame")).toBe(true);
    expect(isSupportedSchema("std_msgs/String")).toBe(false);
    expect(schemaUsesChildFrame("tf2_msgs/msg/TFMessage")).toBe(true);
    expect(schemaUsesChildFrame("nav_msgs/Odometry")).toBe(false);
    expect(schemaUsesChildFrame("geometry_msgs/msg/TransformStamped")).toBe(true);
  });
});
