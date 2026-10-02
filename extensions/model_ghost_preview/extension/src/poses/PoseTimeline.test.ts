import { describe, expect, it } from "vitest";

import { PoseTimeline, type TimedPose } from "./PoseTimeline";
import type { Quat } from "./extractPose";

function pose(tSec: number, position: readonly [number, number, number], orientation?: Quat): TimedPose {
  return {
    tSec,
    position,
    orientation: orientation ?? [0, 0, 0, 1],
    frameId: "map",
  };
}

function quatDot(a: Quat, b: Quat): number {
  return Math.abs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]);
}

describe("PoseTimeline", () => {
  it("orders out-of-order inserts and replaces duplicate timestamps", () => {
    const timeline = new PoseTimeline();
    timeline.insert(pose(5, [5, 0, 0]));
    timeline.insert(pose(1, [1, 0, 0]));
    timeline.insertMany([pose(3, [3, 0, 0]), pose(1, [9, 0, 0]), pose(0, [0, 0, 0])]);
    const path = timeline.path();
    expect(Array.from(path)).toEqual([0, 0, 0, 9, 0, 0, 3, 0, 0, 5, 0, 0]);
    expect(Array.from(timeline.times())).toEqual([0, 1, 3, 5]);
  });

  it("returns undefined before the first sample and clamps after the last", () => {
    const timeline = new PoseTimeline();
    timeline.insert(pose(2, [2, 0, 0]));
    timeline.insert(pose(4, [4, 0, 0]));
    expect(timeline.sample(1.9, "interpolate")).toBeUndefined();
    expect(timeline.sample(1.9, "previous")).toBeUndefined();
    expect(timeline.sample(10, "interpolate")?.position).toEqual([4, 0, 0]);
    expect(timeline.sample(10, "previous")?.position).toEqual([4, 0, 0]);
    expect(timeline.sample(0, "interpolate")).toBeUndefined();
  });

  it("returns the exact sample on a timestamp hit", () => {
    const timeline = new PoseTimeline();
    timeline.insert(pose(0, [0, 0, 0]));
    timeline.insert(pose(10, [10, 2, 0]));
    expect(timeline.sample(10, "interpolate")?.position).toEqual([10, 2, 0]);
    expect(timeline.sample(0, "previous")?.position).toEqual([0, 0, 0]);
  });

  it("lerps position at the midpoint", () => {
    const timeline = new PoseTimeline();
    timeline.insert(pose(0, [0, 0, 0]));
    timeline.insert(pose(10, [10, 4, -2]));
    expect(timeline.sample(5, "interpolate")?.position).toEqual([5, 2, -1]);
  });

  it("uses the previous sample without interpolating", () => {
    const timeline = new PoseTimeline();
    timeline.insert(pose(0, [0, 0, 0]));
    timeline.insert(pose(10, [10, 0, 0]));
    timeline.insert(pose(20, [20, 0, 0]));
    expect(timeline.sample(5, "previous")?.position).toEqual([0, 0, 0]);
    expect(timeline.sample(10, "previous")?.position).toEqual([10, 0, 0]);
    expect(timeline.sample(19.9, "previous")?.position).toEqual([10, 0, 0]);
  });

  it("slerps quaternions on the short path and stays normalized", () => {
    const q90: Quat = [0, 0, Math.sin(Math.PI / 4), Math.cos(Math.PI / 4)];
    const q90Neg: Quat = [0, 0, -q90[2], -q90[3]];
    const direct = new PoseTimeline();
    direct.insert(pose(0, [0, 0, 0]));
    direct.insert(pose(1, [0, 0, 0], q90));
    const flipped = new PoseTimeline();
    flipped.insert(pose(0, [0, 0, 0]));
    flipped.insert(pose(1, [0, 0, 0], q90Neg));
    const a = direct.sample(0.5, "interpolate");
    const b = flipped.sample(0.5, "interpolate");
    expect(a).toBeDefined();
    expect(b).toBeDefined();
    if (!a || !b) {
      return;
    }
    expect(quatDot(a.orientation, b.orientation)).toBeGreaterThan(0.999);
    expect(a.orientation[3]).toBeCloseTo(Math.cos(Math.PI / 8), 5);
    expect(a.orientation[2]).toBeCloseTo(Math.sin(Math.PI / 8), 5);
    expect(Math.hypot(...a.orientation)).toBeCloseTo(1, 6);
  });

  it("slices the path between two times, including interpolated endpoints", () => {
    const timeline = new PoseTimeline();
    timeline.insert(pose(0, [0, 0, 0]));
    timeline.insert(pose(10, [10, 0, 0]));
    timeline.insert(pose(20, [20, 0, 0]));
    expect(Array.from(timeline.pathSlice(5, 15))).toEqual([5, 0, 0, 10, 0, 0, 15, 0, 0]);
    expect(timeline.pathSlice(-5, -1).length).toBe(0);
  });
});
