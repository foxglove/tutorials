import type { PanelExtensionContext } from "@foxglove/extension";
import { useEffect, useLayoutEffect, useRef, useState, type CSSProperties, type ReactElement } from "react";
import { createRoot } from "react-dom/client";

import { PoseTimeline, type ReceiveTime, type TimedPose } from "./poses/PoseTimeline";
import { ChildFrameLock, extractPose } from "./poses/extractPose";
import { disposeObjectTree, GhostScene, type ScenePose } from "./scene/GhostScene";
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
const EMPTY_RECEIVE: readonly ReceiveTime[] = [];

type Trail = {
  points: Float32Array;
  times: Float64Array;
  receiveTimes: readonly ReceiveTime[];
};

type Playback = {
  currentSec: number | undefined;
  previewSec: number | undefined;
  startTime: TimeLike | undefined;
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
  const [childFrames, setChildFrames] = useState<readonly string[]>([]);
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  const sceneHostRef = useRef<HTMLDivElement>(null);
  const sceneRef = useRef<GhostScene | undefined>(undefined);
  const timelineRef = useRef(new PoseTimeline());
  const framedRef = useRef(false);
  const savedConfig = useRef(false);
  const trailRef = useRef<Trail>({ points: EMPTY_POINTS, times: EMPTY_TIMES, receiveTimes: EMPTY_RECEIVE });
  const trailKeyRef = useRef("");

  useLayoutEffect(() => {
    const host = sceneHostRef.current;
    if (!host) {
      return undefined;
    }
    const scene = new GhostScene(host, {
      onPreviewTime: (timeSec) => {
        context.setPreviewTime(timeSec);
      },
      onSeek: (time) => {
        context.seekPlayback?.(time);
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
  }, [context]);

  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  useEffect(() => {
    const topic = config.general.poseTopic;
    context.setDefaultPanelTitle(topic.length > 0 ? `Ghost: ${topic}` : "Model Ghost Preview");
  }, [context, config.general.poseTopic]);

  useEffect(() => {
    if (!savedConfig.current) {
      savedConfig.current = true;
      return;
    }
    context.saveState(config);
  }, [config, context]);

  useEffect(() => {
    context.updatePanelSettingsEditor(
      buildSettingsTree({
        config,
        topics,
        childFrameIds: childFrames,
        actionHandler: (action) => {
          setConfig((previous) => reduceConfig(previous, action));
        },
      }),
    );
  }, [context, config, topics, childFrames]);

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
    const childFrameLock = new ChildFrameLock();
    timelineRef.current = timeline;
    setLoading(true);
    setLoadError(undefined);
    setPoseCount(0);
    setPoseRevision((value) => value + 1);
    setChildFrames([]);

    const unsubscribe = subscribe({
      topic,
      onNewRangeIterator: async (batchIterator) => {
        timeline.clear();
        childFrameLock.reset();
        framedRef.current = false;
        if (!cancelled) {
          setLoading(true);
          setLoadError(undefined);
          setPoseCount(0);
          setPoseRevision((value) => value + 1);
          setChildFrames([]);
        }
        try {
          for await (const batch of batchIterator) {
            if (cancelled) {
              return;
            }
            const extracted: TimedPose[] = [];
            for (const event of batch) {
              const sample = extractPose(event.schemaName, event.message, {
                childFrameId,
                childFrameLock,
              });
              if (!sample) {
                continue;
              }
              extracted.push({
                tSec: toSec(event.receiveTime),
                receiveTime: { sec: event.receiveTime.sec, nsec: event.receiveTime.nsec },
                position: sample.position,
                orientation: sample.orientation,
                frameId: sample.frameId,
              });
            }
            timeline.insertMany(extracted);
            const observed = childFrameLock.observed();
            setChildFrames((previous) => (sameIds(previous, observed) ? previous : observed));
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
          disposeObjectTree(contents);
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
    const delay = config.model.source === "url" ? 400 : 0;
    const timer = window.setTimeout(() => {
      void run();
    }, delay);
    return () => {
      cancelled = true;
      window.clearTimeout(timer);
    };
  }, [
    config.model.source,
    config.model.url,
    config.model.scale,
    config.model.yaw,
    config.model.pitch,
    config.model.roll,
    config.model.truckColor,
  ]);

  useLayoutEffect(() => {
    const scene = sceneRef.current;
    if (!scene) {
      return;
    }
    const timeline = timelineRef.current;
    const trailKey = `${poseRevision}:${config.path.visible ? "shown" : "hidden"}`;
    if (trailKeyRef.current !== trailKey) {
      trailKeyRef.current = trailKey;
      trailRef.current = config.path.visible
        ? {
            points: timeline.path(),
            times: timeline.times(),
            receiveTimes: timeline.receiveTimes(),
          }
        : { points: EMPTY_POINTS, times: EMPTY_TIMES, receiveTimes: EMPTY_RECEIVE };
    }
    const trail = trailRef.current;
    scene.setColorScheme(colorScheme);
    scene.setGridVisibility(config.view.showGrid ? "shown" : "hidden");
    scene.setGhostAppearance({
      style: config.ghost.style,
      color: config.ghost.color,
      opacity: config.ghost.opacity,
    });
    scene.setPath({
      points: trail.points,
      times: trail.times,
      receiveTimes: trail.receiveTimes,
      color: config.path.color,
      visibility: config.path.visible ? "shown" : "hidden",
    });

    const currentPose =
      playback.currentSec == undefined
        ? undefined
        : toScenePose(timeline, timeline.sample(playback.currentSec, config.general.interpolation));
    const previewSec = normalizePreviewTime(playback.previewSec, playback.startTime);
    let ghostPose =
      config.ghost.visible && previewSec != undefined
        ? toScenePose(timeline, timeline.sample(previewSec, config.general.interpolation))
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
        points: timeline.pathSlice(playback.currentSec, previewSec, config.general.interpolation),
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
    if (
      config.ghost.showTimeLabel &&
      ghostPose &&
      playback.currentSec != undefined &&
      previewSec != undefined
    ) {
      const delta = previewSec - playback.currentSec;
      const sign = delta >= 0 ? "+" : "";
      scene.setTimeLabel({
        text: `${sign}${delta.toFixed(2)} s`,
        position: ghostPose.position,
      });
    } else {
      scene.setTimeLabel(undefined);
    }
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

function toScenePose(timeline: PoseTimeline, sample: TimedPose | undefined): ScenePose | undefined {
  if (!sample) {
    return undefined;
  }
  return { position: timeline.rebase(sample.position), orientation: sample.orientation };
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

function sameIds(previous: readonly string[], next: readonly string[]): boolean {
  if (previous.length !== next.length) {
    return false;
  }
  for (let index = 0; index < previous.length; index += 1) {
    if (previous[index] !== next[index]) {
      return false;
    }
  }
  return true;
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
  if (args.poseCount === 0) {
    return { text: "No poses on topic", tone: "info" };
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
