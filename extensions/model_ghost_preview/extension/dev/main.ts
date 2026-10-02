import type { PanelExtensionContext } from "@foxglove/extension";

import { initGhostPreviewPanel } from "../src/GhostPreviewPanel";

const EPOCH = 1_700_000_000;
const DURATION = 40;
const RATE = 10;

type Vec3 = { x: number; y: number; z: number };
type Quat = { x: number; y: number; z: number; w: number };

type PoseMessage = {
  topic: string;
  schemaName: string;
  receiveTime: { sec: number; nsec: number };
  message: Record<string, unknown>;
  sizeInBytes: number;
};

function samplePath(t: number): { position: Vec3; orientation: Quat } {
  const length = 96;
  const scale = length / 280;
  const s = t / DURATION;
  const x = s * length;
  const y = scale * (35 * Math.sin(s * Math.PI * 1.5) + 8 * Math.sin(s * Math.PI * 4));
  const z = scale * (6 * s + 1.5 * Math.sin(s * Math.PI * 2));
  const ds = 1 / DURATION;
  const dx = length * ds;
  const dy =
    scale *
    (35 * Math.PI * 1.5 * ds * Math.cos(s * Math.PI * 1.5) +
      8 * Math.PI * 4 * ds * Math.cos(s * Math.PI * 4));
  const dz = scale * (6 * ds + 1.5 * Math.PI * 2 * ds * Math.cos(s * Math.PI * 2));
  const yaw = Math.atan2(dy, dx);
  const pitch = Math.atan2(dz, Math.hypot(dx, dy));
  return { position: { x, y, z }, orientation: headingQuaternion(yaw, -pitch) };
}

function headingQuaternion(yaw: number, pitch: number): Quat {
  const halfYaw = yaw * 0.5;
  const halfPitch = pitch * 0.5;
  const yawQ: Quat = { x: 0, y: 0, z: Math.sin(halfYaw), w: Math.cos(halfYaw) };
  const pitchQ: Quat = { x: 0, y: Math.sin(halfPitch), z: 0, w: Math.cos(halfPitch) };
  return multiplyQuat(yawQ, pitchQ);
}

function multiplyQuat(a: Quat, b: Quat): Quat {
  return {
    x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
  };
}

function toTime(seconds: number): { sec: number; nsec: number } {
  const sec = Math.floor(seconds);
  let nsec = Math.round((seconds - sec) * 1e9);
  if (nsec >= 1e9) {
    return { sec: sec + 1, nsec: 0 };
  }
  return { sec, nsec };
}

function buildMessages(): { pose: PoseMessage[]; tf: PoseMessage[] } {
  const pose: PoseMessage[] = [];
  const tf: PoseMessage[] = [];
  const count = DURATION * RATE;
  for (let index = 0; index < count; index += 1) {
    const t = index / RATE;
    const absolute = EPOCH + t;
    const { position, orientation } = samplePath(t);
    const receiveTime = toTime(absolute);
    pose.push({
      topic: "/truck/pose",
      schemaName: "foxglove.PoseInFrame",
      receiveTime,
      sizeInBytes: 128,
      message: {
        timestamp: receiveTime,
        frame_id: "map",
        pose: { position, orientation },
      },
    });
    tf.push({
      topic: "/tf",
      schemaName: "foxglove.FrameTransforms",
      receiveTime,
      sizeInBytes: 160,
      message: {
        transforms: [
          {
            timestamp: receiveTime,
            parent_frame_id: "map",
            child_frame_id: "truck",
            translation: position,
            rotation: orientation,
          },
        ],
      },
    });
  }
  return { pose, tf };
}

const messages = buildMessages();

function createContext(panelElement: HTMLDivElement): PanelExtensionContext {
  const start = { sec: EPOCH, nsec: 0 };
  const end = { sec: EPOCH + DURATION, nsec: 0 };
  const requestedText = new URLSearchParams(window.location.search).get("t");
  const requested = requestedText == null ? Number.NaN : Number(requestedText);
  let current = EPOCH + (Number.isFinite(requested) ? Math.min(DURATION, Math.max(0, requested)) : 8);
  let preview: number | undefined;
  let playing = false;
  let rendering = false;
  let pending = false;
  let rangeToken = 0;

  const context = {
    panelElement,
    initialState: {
      general: { poseTopic: "/truck/pose", childFrameId: "truck" },
    },
    dataSourceProfile: "foxglove",
    layout: {
      addPanel: () => undefined,
    },
    watch: () => undefined,
    saveState: (state: unknown) => {
      console.log("saveState", state);
    },
    setParameter: () => undefined,
    setSharedPanelState: () => undefined,
    setVariable: () => undefined,
    setPreviewTime: (time: number | undefined) => {
      preview = time;
      emit();
    },
    seekPlayback: (time: number | { sec: number; nsec: number }) => {
      current = typeof time === "number" ? time : time.sec + time.nsec * 1e-9;
      emit();
    },
    subscribe: () => undefined,
    unsubscribeAll: () => undefined,
    subscribeAppSettings: () => undefined,
    updatePanelSettingsEditor: () => undefined,
    setDefaultPanelTitle: () => undefined,
    subscribeMessageRange: (args: {
      topic: string;
      onNewRangeIterator: (batches: AsyncIterable<PoseMessage[]>) => Promise<void>;
    }) => {
      rangeToken += 1;
      const token = rangeToken;
      const data = args.topic === "/tf" ? messages.tf : messages.pose;
      void args.onNewRangeIterator(iterateBatches(data, () => token === rangeToken));
      return () => {
        rangeToken += 1;
      };
    },
  };

  function emit(): void {
    paintTransport();
    if (!context.onRender) {
      return;
    }
    if (rendering) {
      pending = true;
      return;
    }
    rendering = true;
    context.onRender(
      {
        currentTime: toTime(current),
        startTime: start,
        endTime: end,
        previewTime: preview,
        colorScheme: "light",
        topics: [
          {
            name: "/truck/pose",
            datatype: "foxglove.PoseInFrame",
            schemaName: "foxglove.PoseInFrame",
          },
          {
            name: "/tf",
            datatype: "foxglove.FrameTransforms",
            schemaName: "foxglove.FrameTransforms",
          },
          { name: "/notes", datatype: "std_msgs/String", schemaName: "std_msgs/String" },
        ],
      },
      () => {
        rendering = false;
        if (pending) {
          pending = false;
          emit();
        }
      },
    );
  }

  let last = performance.now();
  const tick = (now: number) => {
    if (playing) {
      const dt = (now - last) / 1000;
      current = Math.min(EPOCH + DURATION, current + dt);
      if (current >= EPOCH + DURATION) {
        playing = false;
        const button = document.querySelector("#play");
        if (button) {
          button.textContent = "Play";
        }
      }
      emit();
    }
    last = now;
    requestAnimationFrame(tick);
  };
  requestAnimationFrame(tick);

  const timeline = document.querySelector("#timeline");
  const play = document.querySelector("#play");
  play?.addEventListener("click", () => {
    playing = !playing;
    if (play) {
      play.textContent = playing ? "Pause" : "Play";
    }
    last = performance.now();
  });
  timeline?.addEventListener("mousemove", (event) => {
    if (!(event instanceof MouseEvent) || !timeline) {
      return;
    }
    preview = EPOCH + fractionOf(timeline, event.clientX) * DURATION;
    emit();
  });
  timeline?.addEventListener("mouseleave", () => {
    preview = undefined;
    emit();
  });
  timeline?.addEventListener("click", (event) => {
    if (!(event instanceof MouseEvent) || !timeline) {
      return;
    }
    current = EPOCH + fractionOf(timeline, event.clientX) * DURATION;
    preview = undefined;
    emit();
  });

  const tryEmit = () => {
    if (context.onRender) {
      emit();
      return;
    }
    requestAnimationFrame(tryEmit);
  };
  requestAnimationFrame(tryEmit);

  return context as unknown as PanelExtensionContext;

  function paintTransport(): void {
    const played = document.querySelector("#played");
    const currentMark = document.querySelector("#current-mark");
    const previewMark = document.querySelector("#preview-mark");
    const readout = document.querySelector("#readout");
    const currentFraction = (current - EPOCH) / DURATION;
    if (played instanceof HTMLElement) {
      played.style.width = `${currentFraction * 100}%`;
    }
    if (currentMark instanceof HTMLElement) {
      currentMark.style.left = `${currentFraction * 100}%`;
    }
    if (previewMark instanceof HTMLElement) {
      if (preview === undefined) {
        previewMark.style.display = "none";
      } else {
        previewMark.style.display = "block";
        previewMark.style.left = `${((preview - EPOCH) / DURATION) * 100}%`;
      }
    }
    if (readout) {
      const previewText = preview === undefined ? "—" : (preview - EPOCH).toFixed(2);
      readout.textContent = `t=${(current - EPOCH).toFixed(2)}s  hover=${previewText}s`;
    }
  }
}

function fractionOf(element: Element, clientX: number): number {
  const rect = element.getBoundingClientRect();
  if (rect.width <= 0) {
    return 0;
  }
  return Math.min(1, Math.max(0, (clientX - rect.left) / rect.width));
}

async function* iterateBatches(
  data: PoseMessage[],
  active: () => boolean,
): AsyncGenerator<PoseMessage[]> {
  const batchSize = 40;
  for (let index = 0; index < data.length; index += batchSize) {
    if (!active()) {
      return;
    }
    await new Promise((resolve) => {
      setTimeout(resolve, 12);
    });
    if (!active()) {
      return;
    }
    yield data.slice(index, index + batchSize);
  }
}

const panel = document.querySelector("#panel");
if (!(panel instanceof HTMLDivElement)) {
  throw new Error("Missing #panel");
}
initGhostPreviewPanel(createContext(panel));
