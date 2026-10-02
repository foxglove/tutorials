import type { SettingsTreeAction } from "@foxglove/extension";
import { describe, expect, it } from "vitest";

import { buildSettingsTree, DEFAULT_CONFIG, mergeConfig, reduceConfig } from "./settings";

describe("mergeConfig", () => {
  it("returns defaults for empty or invalid saved state", () => {
    expect(mergeConfig(undefined)).toEqual(DEFAULT_CONFIG);
    expect(mergeConfig(null)).toEqual(DEFAULT_CONFIG);
    expect(mergeConfig("nope")).toEqual(DEFAULT_CONFIG);
  });

  it("overlays partial saved state without dropping sibling defaults", () => {
    const merged = mergeConfig({
      model: { scale: 2.5 },
      ghost: { color: "#ffffff" },
      general: { interpolation: "nope" },
    });
    expect(merged.model.scale).toBe(2.5);
    expect(merged.model.truckColor).toBe(DEFAULT_CONFIG.model.truckColor);
    expect(merged.model.source).toBe("truck");
    expect(merged.ghost.color).toBe("#ffffff");
    expect(merged.ghost.style).toBe("wireframe");
    expect(merged.ghost.visible).toBe(true);
    expect(merged.general.interpolation).toBe("interpolate");
    expect(merged.path.visible).toBe(true);
    expect(merged.view.follow).toBe("off");
  });

  it("does not mutate the default config", () => {
    const merged = mergeConfig({ ghost: { opacity: 0.2 } });
    merged.ghost.opacity = 1;
    merged.model.scale = 4;
    expect(DEFAULT_CONFIG.ghost.opacity).toBe(0.6);
    expect(DEFAULT_CONFIG.model.scale).toBe(1);
  });
});

describe("reduceConfig", () => {
  it("updates nested fields immutably", () => {
    const next = reduceConfig(DEFAULT_CONFIG, {
      action: "update",
      payload: { path: ["model", "yaw"], input: "number", value: 15 },
    });
    expect(next.model.yaw).toBe(15);
    expect(next.model).not.toBe(DEFAULT_CONFIG.model);
    expect(DEFAULT_CONFIG.model.yaw).toBe(0);
    expect(next.general).toBe(DEFAULT_CONFIG.general);
  });

  it("toggles node visibility from the visibility path", () => {
    const hidden = reduceConfig(DEFAULT_CONFIG, {
      action: "update",
      payload: { path: ["ghost", "visibility"], input: "boolean", value: false },
    });
    expect(hidden.ghost.visible).toBe(false);
    const shown = reduceConfig(hidden, {
      action: "update",
      payload: { path: ["path", "visible"], input: "boolean", value: false },
    });
    expect(shown.path.visible).toBe(false);
    expect(shown.ghost.visible).toBe(false);
  });

  it("ignores unknown actions and paths", () => {
    const action: SettingsTreeAction = {
      action: "perform-node-action",
      payload: { id: "reset", path: ["general"] },
    };
    expect(reduceConfig(DEFAULT_CONFIG, action)).toBe(DEFAULT_CONFIG);
    expect(
      reduceConfig(DEFAULT_CONFIG, {
        action: "update",
        payload: { path: ["missing", "field"], input: "string", value: "x" },
      }),
    ).toBe(DEFAULT_CONFIG);
  });

  it("clamps opacity and rejects invalid enums", () => {
    const next = reduceConfig(DEFAULT_CONFIG, {
      action: "update",
      payload: { path: ["ghost", "opacity"], input: "number", value: 4 },
    });
    expect(next.ghost.opacity).toBe(1);
    expect(
      reduceConfig(DEFAULT_CONFIG, {
        action: "update",
        payload: { path: ["view", "follow"], input: "select", value: "sideways" },
      }).view.follow,
    ).toBe("off");
  });
});

describe("buildSettingsTree", () => {
  it("lists supported topics and shows the child frame field for TF schemas", () => {
    const tree = buildSettingsTree({
      config: mergeConfig({ general: { poseTopic: "/tf" } }),
      topics: [
        { name: "/truck/pose", schemaName: "foxglove.PoseInFrame" },
        { name: "/tf", schemaName: "foxglove.FrameTransforms" },
        { name: "/notes", schemaName: "std_msgs/String" },
      ],
      childFrameIds: ["base", "truck"],
      actionHandler: () => undefined,
    });
    const general = tree.nodes["general"];
    const poseField = general?.fields?.["poseTopic"];
    expect(poseField?.input).toBe("select");
    if (poseField?.input !== "select") {
      return;
    }
    const labels = poseField.options.map((option) => option.label);
    expect(labels).toEqual(["/truck/pose", "/tf"]);
    const childField = general?.fields?.["childFrameId"];
    expect(childField?.input).toBe("autocomplete");
    if (childField?.input === "autocomplete") {
      expect(childField.items).toEqual(["base", "truck"]);
    }
    expect(tree.nodes["ghost"]?.visible).toBe(true);
    expect(tree.nodes["ghost"]?.label).toBe("Preview");
  });
});
