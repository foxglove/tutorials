import type { PanelExtensionContext } from "@foxglove/extension";
import { useEffect, useLayoutEffect, useRef, useState, type CSSProperties, type ReactElement } from "react";
import { createRoot } from "react-dom/client";

import { PoseTimeline, type TimedPose } from "./poses/PoseTimeline";
import { extractPose } from "./poses/extractPose";
import { GhostScene, type ScenePose } from "./scene/GhostScene";
import { loadGltfScene, placeModel } from "./scene/placeModel";
import { createTruckModel } from "./scene/truckModel";
import {
  buildSettingsTree,
  mergeConfig,
  reduceConfig,
  type GhostPreviewConfig,
  type TopicInfo,
} from "./settings";
import { normalizePreviewTime, toSec, type TimeLike } from "./time";

const EMPTY_POINTS = new Float32Array();
const EMPTY_TIMES = new Float64Array();

type Playback = {
  currentSec: number | undefined;
  previewSec: number | undefined;
  startTime: TimeLike | undefined;
};

type GhostLabel = {
  text: string;
  x: number;
  y: number;
};

export function GhostPreviewPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [config, setConfig] = useState<GhostPreviewConfig>(() => mergeConfig(context.initialState));
  const [topics, setTopics] = useState<readonly TopicInfo[]>([]);
  const [colorScheme, setColorScheme] = useState<"dark" | "light">("dark");
  const [playback, setPlayback] = useState<Playback>({
    currentSec: undefined,
    previewSec: undefined,
    startTime: undefined,
  });
  const [poseCount, setPoseCount] = useState(0);
  const [poseRevision, setPoseRevision] = useState(0);
  const [loading, setLoading] = useState(false);
  const [loadError, setLoadError] = useState<string | undefined>();
  const [modelError, setModelError] = useState<string | undefined>();
  const [ghostLabel, setGhostLabel] = useState<GhostLabel | undefined>();
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  const sceneHostRef = useRef<HTMLDivElement>(null);
  const sceneRef = useRef<GhostScene | undefined>(undefined);
  const timelineRef = useRef(new PoseTimeline());
  const framedRef = useRef(false);

  useLayoutEffect(() => {
    const host = sceneHostRef.current;
    if (!host) {
      return undefined;
    }
    const scene = new GhostScene(host, {
      onPreviewTime: (timeSec) => {
        context.setPreviewTime(timeSec);
      },
      onSeek: (timeSec) => {
        context.seekPlayback?.(timeSec);
      },
    });
    sceneRef.current = scene;
    return () => {
      scene.dispose();
      sceneRef.current = undefined;
    };
  }, [context]);

  useLayoutEffect(() => {
    context.onRender = (renderState, done) => {
      setRenderDone(() => done);
      if (renderState.topics) {
        setTopics(renderState.topics);
      }
      if (renderState.colorScheme) {
        setColorScheme(renderState.colorScheme);
      }
      const currentSec = renderState.currentTime ? toSec(renderState.currentTime) : undefined;
      const startTime = renderState.startTime ? toTimeLike(renderState.startTime) : undefined;
      setPlayback((previous) => {
        if (
          previous.currentSec === currentSec &&
          previous.previewSec === renderState.previewTime &&
          sameTime(previous.startTime, startTime)
        ) {
          return previous;
        }
        return { currentSec, previewSec: renderState.previewTime, startTime };
      });
    };
    context.watch("currentTime");
    context.watch("previewTime");
    context.watch("topics");
    context.watch("colorScheme");
    context.watch("startTime");
    context.watch("endTime");
  }, [context]);

  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  useEffect(() => {
    const topic = config.general.poseTopic;
    context.setDefaultPanelTitle(topic.length > 0 ? `Ghost: ${topic}` : "Model Ghost Preview");
  }, [context, config.general.poseTopic]);

  useEffect(() => {
    context.updatePanelSettingsEditor(
      buildSettingsTree({
        config,
        topics,
        actionHandler: (action) => {
          setConfig((previous) => {
            const next = reduceConfig(previous, action);
            if (next !== previous) {
              context.saveState(next);
            }
            return next;
          });
        },
      }),
    );
  }, [context, config, topics]);

  useEffect(() => {
    const topic = config.general.poseTopic;
    const childFrameId = config.general.childFrameId;
    const subscribe = context.subscribeMessageRange;
    framedRef.current = false;
    if (topic.length === 0 || !subscribe) {
      timelineRef.current = new PoseTimeline();
      setPoseCount(0);
      setPoseRevision((value) => value + 1);
      setLoading(false);
      setLoadError(
        topic.length > 0 && !subscribe
          ? "Range loading is not available for this data source."
          : undefined,
      );
      return undefined;
    }

    let cancelled = false;
    const timeline = new PoseTimeline();
    timelineRef.current = timeline;
    setLoading(true);
    setLoadError(undefined);
    setPoseCount(0);

    const unsubscribe = subscribe({
      topic,
      onNewRangeIterator: async (batchIterator) => {
        timeline.clear();
        if (!cancelled) {
          setPoseCount(0);
          setPoseRevision((value) => value + 1);
        }
        try {
          for await (const batch of batchIterator) {
            if (cancelled) {
              return;
            }
            const extracted: TimedPose[] = [];
            for (const event of batch) {
              const sample = extractPose(event.schemaName, event.message, { childFrameId });
              if (!sample) {
                continue;
              }
              extracted.push({
                tSec: toSec(event.receiveTime),
                position: sample.position,
                orientation: sample.orientation,
                frameId: sample.frameId,
              });
            }
            timeline.insertMany(extracted);
            setPoseCount(timeline.size());
            setPoseRevision((value) => value + 1);
          }
        } catch (error: unknown) {
          if (!cancelled) {
            setLoadError(error instanceof Error ? error.message : "Failed to load poses");
          }
        } finally {
          if (!cancelled) {
            setLoading(false);
          }
        }
      },
    });

    return () => {
      cancelled = true;
      unsubscribe();
    };
  }, [context, config.general.poseTopic, config.general.childFrameId]);

  useEffect(() => {
    const scene = sceneRef.current;
    if (!scene) {
      return undefined;
    }
    let cancelled = false;
    const placement = {
      scale: config.model.scale,
      yawDeg: config.model.yaw,
      pitchDeg: config.model.pitch,
      rollDeg: config.model.roll,
      yUpCorrection: config.model.source === "url" ? ("apply" as const) : ("skip" as const),
    };
    const run = async (): Promise<void> => {
      try {
        const contents =
          config.model.source === "url"
            ? await loadGltfScene(config.model.url)
            : createTruckModel(config.model.truckColor);
        if (cancelled) {
          return;
        }
        scene.setModel(placeModel(contents, placement));
        setModelError(undefined);
      } catch (error: unknown) {
        if (cancelled) {
          return;
        }
        setModelError(error instanceof Error ? error.message : "Failed to load model");
        scene.setModel(
          placeModel(createTruckModel(config.model.truckColor), {
            ...placement,
            yUpCorrection: "skip",
          }),
        );
      }
    };
    void run();
    return () => {
      cancelled = true;
    };
  }, [
    config.model.source,
    config.model.url,
    config.model.scale,
    config.model.yaw,
    config.model.pitch,
    config.model.roll,
    config.model.truckColor,
    context,
  ]);

  useLayoutEffect(() => {
    const scene = sceneRef.current;
    if (!scene) {
      return;
    }
    const timeline = timelineRef.current;
    scene.setColorScheme(colorScheme);
    scene.setGridVisibility(config.view.showGrid ? "shown" : "hidden");
    scene.setGhostAppearance({
      style: config.ghost.style,
      color: config.ghost.color,
      opacity: config.ghost.opacity,
    });
    scene.setPath({
      points: config.path.visible ? timeline.path() : EMPTY_POINTS,
      times: config.path.visible ? timeline.times() : EMPTY_TIMES,
      color: config.path.color,
      visibility: config.path.visible ? "shown" : "hidden",
    });

    const currentPose =
      playback.currentSec == undefined
        ? undefined
        : toScenePose(timeline.sample(playback.currentSec, config.general.interpolation));
    const previewSec = normalizePreviewTime(playback.previewSec, playback.startTime);
    let ghostPose =
      config.ghost.visible && previewSec != undefined
        ? toScenePose(timeline.sample(previewSec, config.general.interpolation))
        : undefined;
    if (ghostPose && currentPose && posesNearlyEqual(ghostPose, currentPose)) {
      ghostPose = undefined;
    }
    if (
      config.path.visible &&
      config.path.highlightSegment &&
      playback.currentSec != undefined &&
      previewSec != undefined &&
      ghostPose
    ) {
      scene.setHighlight({
        points: timeline.pathSlice(playback.currentSec, previewSec),
        color: config.path.color,
      });
    } else {
      scene.setHighlight(undefined);
    }
    scene.setCurrentPose(currentPose);
    scene.setGhostPose(ghostPose);
    if (!framedRef.current && !loading && timeline.size() > 1) {
      scene.framePath();
      framedRef.current = true;
    }
    scene.setFollowMode(config.view.follow);

    const label =
      config.ghost.showTimeLabel &&
      ghostPose &&
      playback.currentSec != undefined &&
      previewSec != undefined
        ? projectLabel(scene, ghostPose, previewSec - playback.currentSec)
        : undefined;
    setGhostLabel((previous) => (sameLabel(previous, label) ? previous : label));
  }, [colorScheme, config, loading, playback, poseRevision]);

  const status = statusMessage({
    topic: config.general.poseTopic,
    loading,
    poseCount,
    loadError,
    modelError,
  });

  return (
    <div
      style={rootStyle}
      data-pose-count={poseCount}
      data-loading={loading ? "true" : "false"}
      data-model-error={modelError ?? ""}
    >
      <div ref={sceneHostRef} style={hostStyle} />
      {status && <div style={status.tone === "error" ? errorStyle : overlayStyle}>{status.text}</div>}
      {ghostLabel && (
        <div style={{ ...labelStyle, left: ghostLabel.x, top: ghostLabel.y }}>{ghostLabel.text}</div>
      )}
    </div>
  );
}

export function initGhostPreviewPanel(context: PanelExtensionContext): () => void {
  context.panelElement.style.height = "100%";
  context.panelElement.style.width = "100%";
  context.panelElement.style.overflow = "hidden";
  const root = createRoot(context.panelElement);
  root.render(<GhostPreviewPanel context={context} />);
  return () => {
    root.unmount();
  };
}

function toScenePose(sample: TimedPose | undefined): ScenePose | undefined {
  if (!sample) {
    return undefined;
  }
  return { position: sample.position, orientation: sample.orientation };
}

function posesNearlyEqual(a: ScenePose, b: ScenePose): boolean {
  const dx = a.position[0] - b.position[0];
  const dy = a.position[1] - b.position[1];
  const dz = a.position[2] - b.position[2];
  if (dx * dx + dy * dy + dz * dz > 0.0025) {
    return false;
  }
  const dot = Math.abs(
    a.orientation[0] * b.orientation[0] +
      a.orientation[1] * b.orientation[1] +
      a.orientation[2] * b.orientation[2] +
      a.orientation[3] * b.orientation[3],
  );
  return dot > 0.9995;
}

function projectLabel(scene: GhostScene, pose: ScenePose, deltaSec: number): GhostLabel | undefined {
  const point = scene.projectToOverlay(pose.position, 4.6);
  if (!point) {
    return undefined;
  }
  const sign = deltaSec >= 0 ? "+" : "";
  return { text: `${sign}${deltaSec.toFixed(2)} s`, x: point.x, y: point.y };
}

function sameLabel(previous: GhostLabel | undefined, next: GhostLabel | undefined): boolean {
  if (previous == undefined || next == undefined) {
    return previous === next;
  }
  return (
    previous.text === next.text &&
    Math.round(previous.x) === Math.round(next.x) &&
    Math.round(previous.y) === Math.round(next.y)
  );
}

function statusMessage(args: {
  topic: string;
  loading: boolean;
  poseCount: number;
  loadError: string | undefined;
  modelError: string | undefined;
}): { text: string; tone: "info" | "error" } | undefined {
  if (args.modelError) {
    return { text: args.modelError, tone: "error" };
  }
  if (args.loadError) {
    return { text: args.loadError, tone: "error" };
  }
  if (args.topic.length === 0) {
    return { text: "Select a pose topic", tone: "info" };
  }
  if (args.loading) {
    return { text: `loading ${args.poseCount} poses…`, tone: "info" };
  }
  return undefined;
}

function toTimeLike(time: TimeLike): TimeLike {
  return { sec: time.sec, nsec: time.nsec };
}

function sameTime(a: TimeLike | undefined, b: TimeLike | undefined): boolean {
  if (a == undefined || b == undefined) {
    return a === b;
  }
  return a.sec === b.sec && a.nsec === b.nsec;
}

const rootStyle: CSSProperties = {
  position: "relative",
  width: "100%",
  height: "100%",
  overflow: "hidden",
  fontFamily: "system-ui, sans-serif",
};

const hostStyle: CSSProperties = { position: "absolute", inset: 0 };

const overlayStyle: CSSProperties = {
  position: "absolute",
  left: 8,
  top: 8,
  padding: "4px 8px",
  borderRadius: 4,
  background: "rgba(20, 24, 28, 0.72)",
  color: "#f2f5f8",
  fontSize: 12,
  pointerEvents: "none",
};

const errorStyle: CSSProperties = {
  ...overlayStyle,
  background: "rgba(120, 32, 32, 0.88)",
  maxWidth: "70%",
};

const labelStyle: CSSProperties = {
  position: "absolute",
  transform: "translate(-50%, -130%)",
  padding: "2px 6px",
  borderRadius: 4,
  background: "rgba(12, 32, 48, 0.8)",
  color: "#d7f3ff",
  fontSize: 12,
  pointerEvents: "none",
  whiteSpace: "nowrap",
};
