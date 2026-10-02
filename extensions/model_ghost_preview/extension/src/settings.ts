import type { SettingsTree, SettingsTreeAction, SettingsTreeNode } from "@foxglove/extension";

import { isSupportedSchema, schemaUsesChildFrame } from "./poses/extractPose";

export type InterpolationMode = "interpolate" | "previous";
export type ModelSource = "truck" | "url";
export type GhostStyle = "wireframe" | "transparent" | "solid";
export type FollowMode = "off" | "current" | "ghost";

export type GhostPreviewConfig = {
  general: {
    poseTopic: string;
    childFrameId: string;
    interpolation: InterpolationMode;
  };
  model: {
    source: ModelSource;
    url: string;
    scale: number;
    yaw: number;
    pitch: number;
    roll: number;
    truckColor: string;
  };
  ghost: {
    visible: boolean;
    style: GhostStyle;
    color: string;
    opacity: number;
    showTimeLabel: boolean;
  };
  path: {
    visible: boolean;
    color: string;
    highlightSegment: boolean;
  };
  view: {
    follow: FollowMode;
    showGrid: boolean;
  };
};

export type TopicInfo = {
  name: string;
  schemaName: string;
};

export const DEFAULT_CONFIG: GhostPreviewConfig = {
  general: {
    poseTopic: "",
    childFrameId: "",
    interpolation: "interpolate",
  },
  model: {
    source: "truck",
    url: "",
    scale: 1,
    yaw: 0,
    pitch: 0,
    roll: 0,
    truckColor: "#e6b422",
  },
  ghost: {
    visible: true,
    style: "wireframe",
    color: "#4fc3f7",
    opacity: 0.6,
    showTimeLabel: true,
  },
  path: {
    visible: true,
    color: "#2b3cff",
    highlightSegment: true,
  },
  view: {
    follow: "off",
    showGrid: true,
  },
};

const INTERPOLATION_MODES = ["interpolate", "previous"] as const;
const MODEL_SOURCES = ["truck", "url"] as const;
const GHOST_STYLES = ["wireframe", "transparent", "solid"] as const;
const FOLLOW_MODES = ["off", "current", "ghost"] as const;

export function mergeConfig(saved: unknown): GhostPreviewConfig {
  const record = asRecord(saved);
  return {
    general: mergeGeneral(record?.["general"]),
    model: mergeModel(record?.["model"]),
    ghost: mergeGhost(record?.["ghost"]),
    path: mergePath(record?.["path"]),
    view: mergeView(record?.["view"]),
  };
}

export function reduceConfig(config: GhostPreviewConfig, action: SettingsTreeAction): GhostPreviewConfig {
  switch (action.action) {
    case "update":
      return applyUpdate(config, action.payload.path, action.payload.value);
    case "perform-node-action":
    case "reorder-children":
      return config;
    default: {
      const unexpected: never = action;
      return unexpected;
    }
  }
}

export function buildSettingsTree(args: {
  config: GhostPreviewConfig;
  topics: readonly TopicInfo[];
  actionHandler: (action: SettingsTreeAction) => void;
}): SettingsTree {
  const { config, topics, actionHandler } = args;
  const supported = topics.filter((topic) => isSupportedSchema(topic.schemaName));
  const options = supported.map((topic) => ({ label: topic.name, value: topic.name }));
  if (config.general.poseTopic.length > 0 && !options.some((option) => option.value === config.general.poseTopic)) {
    options.unshift({ label: config.general.poseTopic, value: config.general.poseTopic });
  }
  if (options.length === 0) {
    options.push({ label: "No pose topics", value: "" });
  }
  const selected = topics.find((topic) => topic.name === config.general.poseTopic);
  const showChildFrame = selected ? schemaUsesChildFrame(selected.schemaName) : false;

  const general: SettingsTreeNode = {
    label: "General",
    icon: "Topic",
    order: 0,
    fields: {
      poseTopic: {
        label: "Pose topic",
        input: "select",
        value: options.some((option) => option.value === config.general.poseTopic)
          ? config.general.poseTopic
          : "",
        options,
        help: "Poses are taken in this topic's frame. Receive time is the timeline key.",
      },
      interpolation: {
        label: "Interpolation",
        input: "toggle",
        value: config.general.interpolation,
        options: [
          { label: "Interpolate", value: "interpolate" },
          { label: "Previous", value: "previous" },
        ],
      },
      ...(showChildFrame
        ? {
            childFrameId: {
              label: "Child frame",
              input: "string" as const,
              value: config.general.childFrameId,
              placeholder: "first transform",
              help: "Matches child_frame_id. Leave empty to use the first transform.",
            },
          }
        : {}),
    },
  };

  const model: SettingsTreeNode = {
    label: "Model",
    icon: "PrecisionManufacturing",
    order: 1,
    fields: {
      source: {
        label: "Source",
        input: "toggle",
        value: config.model.source,
        options: [
          { label: "Truck", value: "truck" },
          { label: "URL", value: "url" },
        ],
      },
      url: {
        label: "Model URL",
        input: "string",
        value: config.model.url,
        placeholder: "https://example.com/robot.glb",
        disabled: config.model.source !== "url",
        help: "glTF is Y-up. A +90° X rotation is applied before the offsets below; set roll to -90 to cancel it.",
      },
      scale: {
        label: "Scale",
        input: "number",
        value: config.model.scale,
        min: 0.01,
        step: 0.1,
        precision: 3,
      },
      yaw: numberField("Yaw", config.model.yaw, "deg"),
      pitch: numberField("Pitch", config.model.pitch, "deg"),
      roll: numberField("Roll", config.model.roll, "deg"),
      truckColor: {
        label: "Truck color",
        input: "rgb",
        value: config.model.truckColor,
        disabled: config.model.source !== "truck",
      },
    },
  };

  const ghost: SettingsTreeNode = {
    label: "Preview",
    icon: "Shapes",
    order: 2,
    visible: config.ghost.visible,
    help: "Ghost of the model at the hovered timeline time.",
    fields: {
      style: {
        label: "Style",
        input: "select",
        value: config.ghost.style,
        options: [
          { label: "Wireframe", value: "wireframe" },
          { label: "Transparent", value: "transparent" },
          { label: "Solid", value: "solid" },
        ],
      },
      color: { label: "Color", input: "rgb", value: config.ghost.color },
      opacity: {
        label: "Opacity",
        input: "number",
        value: config.ghost.opacity,
        min: 0,
        max: 1,
        step: 0.05,
        precision: 2,
      },
      showTimeLabel: {
        label: "Time label",
        input: "boolean",
        value: config.ghost.showTimeLabel,
      },
    },
  };

  const path: SettingsTreeNode = {
    label: "Path",
    icon: "Timeline",
    order: 3,
    visible: config.path.visible,
    fields: {
      color: { label: "Color", input: "rgb", value: config.path.color },
      highlightSegment: {
        label: "Highlight segment",
        input: "boolean",
        value: config.path.highlightSegment,
        help: "Brighten the path between the current time and the preview time.",
      },
    },
  };

  const view: SettingsTreeNode = {
    label: "View",
    icon: "World",
    order: 4,
    fields: {
      follow: {
        label: "Follow",
        input: "select",
        value: config.view.follow,
        options: [
          { label: "Off", value: "off" },
          { label: "Current", value: "current" },
          { label: "Ghost", value: "ghost" },
        ],
      },
      showGrid: {
        label: "Grid",
        input: "boolean",
        value: config.view.showGrid,
      },
    },
  };

  return {
    actionHandler,
    enableFilter: false,
    nodes: { general, model, ghost, path, view },
  };
}

function numberField(label: string, value: number, suffix: string): {
  label: string;
  input: "number";
  value: number;
  step: number;
  precision: number;
  placeholder: string;
} {
  return { label, input: "number", value, step: 1, precision: 1, placeholder: suffix };
}

function applyUpdate(
  config: GhostPreviewConfig,
  path: readonly string[],
  value: unknown,
): GhostPreviewConfig {
  const section = path[0];
  const key = path[1];
  if (!section || !key) {
    return config;
  }
  if ((key === "visible" || key === "visibility") && typeof value === "boolean") {
    if (section === "ghost") {
      return { ...config, ghost: { ...config.ghost, visible: value } };
    }
    if (section === "path") {
      return { ...config, path: { ...config.path, visible: value } };
    }
  }
  if (section === "general") {
    if (key === "poseTopic" && typeof value === "string") {
      return { ...config, general: { ...config.general, poseTopic: value } };
    }
    if (key === "childFrameId" && typeof value === "string") {
      return { ...config, general: { ...config.general, childFrameId: value } };
    }
    if (key === "interpolation" && isOneOf(value, INTERPOLATION_MODES)) {
      return { ...config, general: { ...config.general, interpolation: value } };
    }
  }
  if (section === "model") {
    if (key === "source" && isOneOf(value, MODEL_SOURCES)) {
      return { ...config, model: { ...config.model, source: value } };
    }
    if (key === "url" && typeof value === "string") {
      return { ...config, model: { ...config.model, url: value } };
    }
    if (key === "scale" && isFiniteNumber(value)) {
      return { ...config, model: { ...config.model, scale: value } };
    }
    if (key === "yaw" && isFiniteNumber(value)) {
      return { ...config, model: { ...config.model, yaw: value } };
    }
    if (key === "pitch" && isFiniteNumber(value)) {
      return { ...config, model: { ...config.model, pitch: value } };
    }
    if (key === "roll" && isFiniteNumber(value)) {
      return { ...config, model: { ...config.model, roll: value } };
    }
    if (key === "truckColor" && typeof value === "string") {
      return { ...config, model: { ...config.model, truckColor: value } };
    }
  }
  if (section === "ghost") {
    if (key === "style" && isOneOf(value, GHOST_STYLES)) {
      return { ...config, ghost: { ...config.ghost, style: value } };
    }
    if (key === "color" && typeof value === "string") {
      return { ...config, ghost: { ...config.ghost, color: value } };
    }
    if (key === "opacity" && isFiniteNumber(value)) {
      return { ...config, ghost: { ...config.ghost, opacity: clamp(value, 0, 1) } };
    }
    if (key === "showTimeLabel" && typeof value === "boolean") {
      return { ...config, ghost: { ...config.ghost, showTimeLabel: value } };
    }
  }
  if (section === "path") {
    if (key === "color" && typeof value === "string") {
      return { ...config, path: { ...config.path, color: value } };
    }
    if (key === "highlightSegment" && typeof value === "boolean") {
      return { ...config, path: { ...config.path, highlightSegment: value } };
    }
  }
  if (section === "view") {
    if (key === "follow" && isOneOf(value, FOLLOW_MODES)) {
      return { ...config, view: { ...config.view, follow: value } };
    }
    if (key === "showGrid" && typeof value === "boolean") {
      return { ...config, view: { ...config.view, showGrid: value } };
    }
  }
  return config;
}

function mergeGeneral(value: unknown): GhostPreviewConfig["general"] {
  const record = asRecord(value);
  const fallback = DEFAULT_CONFIG.general;
  if (!record) {
    return { ...fallback };
  }
  return {
    poseTopic: pickString(record["poseTopic"], fallback.poseTopic),
    childFrameId: pickString(record["childFrameId"], fallback.childFrameId),
    interpolation: pickEnum(record["interpolation"], INTERPOLATION_MODES, fallback.interpolation),
  };
}

function mergeModel(value: unknown): GhostPreviewConfig["model"] {
  const record = asRecord(value);
  const fallback = DEFAULT_CONFIG.model;
  if (!record) {
    return { ...fallback };
  }
  return {
    source: pickEnum(record["source"], MODEL_SOURCES, fallback.source),
    url: pickString(record["url"], fallback.url),
    scale: pickNumber(record["scale"], fallback.scale),
    yaw: pickNumber(record["yaw"], fallback.yaw),
    pitch: pickNumber(record["pitch"], fallback.pitch),
    roll: pickNumber(record["roll"], fallback.roll),
    truckColor: pickString(record["truckColor"], fallback.truckColor),
  };
}

function mergeGhost(value: unknown): GhostPreviewConfig["ghost"] {
  const record = asRecord(value);
  const fallback = DEFAULT_CONFIG.ghost;
  if (!record) {
    return { ...fallback };
  }
  return {
    visible: typeof record["visible"] === "boolean" ? record["visible"] : fallback.visible,
    style: pickEnum(record["style"], GHOST_STYLES, fallback.style),
    color: pickString(record["color"], fallback.color),
    opacity: clamp(pickNumber(record["opacity"], fallback.opacity), 0, 1),
    showTimeLabel:
      typeof record["showTimeLabel"] === "boolean" ? record["showTimeLabel"] : fallback.showTimeLabel,
  };
}

function mergePath(value: unknown): GhostPreviewConfig["path"] {
  const record = asRecord(value);
  const fallback = DEFAULT_CONFIG.path;
  if (!record) {
    return { ...fallback };
  }
  return {
    visible: typeof record["visible"] === "boolean" ? record["visible"] : fallback.visible,
    color: pickString(record["color"], fallback.color),
    highlightSegment:
      typeof record["highlightSegment"] === "boolean"
        ? record["highlightSegment"]
        : fallback.highlightSegment,
  };
}

function mergeView(value: unknown): GhostPreviewConfig["view"] {
  const record = asRecord(value);
  const fallback = DEFAULT_CONFIG.view;
  if (!record) {
    return { ...fallback };
  }
  return {
    follow: pickEnum(record["follow"], FOLLOW_MODES, fallback.follow),
    showGrid: typeof record["showGrid"] === "boolean" ? record["showGrid"] : fallback.showGrid,
  };
}

function pickString(value: unknown, fallback: string): string {
  return typeof value === "string" ? value : fallback;
}

function pickNumber(value: unknown, fallback: number): number {
  return isFiniteNumber(value) ? value : fallback;
}

function pickEnum<T extends string>(value: unknown, allowed: readonly T[], fallback: T): T {
  if (typeof value !== "string") {
    return fallback;
  }
  for (const option of allowed) {
    if (option === value) {
      return option;
    }
  }
  return fallback;
}

function isOneOf<T extends string>(value: unknown, allowed: readonly T[]): value is T {
  if (typeof value !== "string") {
    return false;
  }
  for (const option of allowed) {
    if (option === value) {
      return true;
    }
  }
  return false;
}

function isFiniteNumber(value: unknown): value is number {
  return typeof value === "number" && Number.isFinite(value);
}

function clamp(value: number, min: number, max: number): number {
  return Math.min(max, Math.max(min, value));
}

function asRecord(value: unknown): Record<string, unknown> | undefined {
  if (typeof value !== "object" || value == null) {
    return undefined;
  }
  return value as Record<string, unknown>;
}
