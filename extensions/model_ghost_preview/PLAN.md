# Model Ghost Preview extension — implementation plan

Source request: Linear VIZ-3444, "Preview topic visualization (e.g. 3D model) at hovered timeline
position while paused". Community ask: while hovering/skimming the playback timeline, render a
"ghost" of a topic's 3D model at the hovered time, alongside the normal render at the current time
(like the Map panel's GNSS hover preview). Reference screenshot: a textured haul truck at the current
time, plus a blue/cyan wireframe copy of the same truck further along its blue path, at the hovered
time. Proposed UX: per-topic "Preview ON | OFF" plus style overrides (wireframe, color, opacity).

## Feasibility findings (already verified against `@foxglove/extension` 3.3.0 types)

- `RenderState.previewTime?: number` — "set when a user hovers over the seek bar or when a panel sets
  the preview time explicitly". It is a seconds value. In Foxglove it is absolute seconds
  (`toSec(startTime) + hoverOffset`), but be defensive: if `previewTime < toSec(startTime) - 1` and
  `startTime` is non-trivial, treat it as relative and add `toSec(startTime)`. Requires
  `context.watch("previewTime")`.
- `context.setPreviewTime(sec | undefined)` and `context.seekPlayback?.(time)` exist, so the panel can
  also drive the preview/seek itself.
- `context.subscribeMessageRange({ topic, onNewRangeIterator })` gives the whole recording's messages
  for a topic (batches in log-time order, best effort). This is what lets us look up a pose at an
  arbitrary hovered time while playback is paused. (`preload`/`allFrames` is deprecated; do not use.)
- Why not a message converter feeding the built-in 3D panel: topic converters only run when a new
  input message arrives, and "global variables do not cause re-processing of current frame data".
  While paused nothing arrives, so a converter cannot move a ghost as the user hovers. Built-in 3D
  panel internals are not extensible. Therefore: a **custom panel** that renders its own 3D scene with
  three.js. Document this trade-off in the README (and that native support in the 3D panel is what
  VIZ-3444 ultimately asks for; this extension is a working stop-gap / prototype of the UX).

## Deliverables (all under `extensions/model_ghost_preview/`)

```
extensions/model_ghost_preview/
  README.md                # with YAML front-matter (title, short_description) — see CONTRIBUTING
  PLAN.md                  # this file
  extension/               # the Foxglove extension (scaffolded with create-foxglove-extension)
    package.json, package-lock.json, tsconfig.json, eslint.config.js, .prettierrc.yaml, .gitignore,
    LICENSE (MIT), CHANGELOG.md, README.md (short, extension-marketplace style)
    src/index.ts           # activate(): registerPanel({ name: "Model Ghost Preview", initPanel })
    src/GhostPreviewPanel.tsx   # React panel: context wiring, settings tree, render loop glue
    src/scene/GhostScene.ts     # three.js scene/camera/renderer/controls; no Foxglove imports
    src/scene/truckModel.ts     # procedural haul-truck model (default model; no external asset)
    src/scene/ghostMaterial.ts  # apply ghost style (wireframe / transparent / solid) to an Object3D clone
    src/poses/extractPose.ts    # message -> {position, orientation, frameId} for supported schemas
    src/poses/PoseTimeline.ts   # sorted timestamps + poses; binary search; lerp/slerp interpolation
    src/settings.ts             # config type, defaults, settings-tree builder, action reducer
    src/time.ts                 # Time <-> seconds helpers, previewTime normalization
    src/*.test.ts               # unit tests (see Testing)
    dev/                        # standalone browser harness (see Testing) — NOT part of extension bundle
  demo/
    generate_demo_mcap.py  # writes demo.mcap with foxglove SDK (pip `foxglove-sdk`)
    requirements.txt
  foxglove_layouts/model_ghost_preview.json   # layout: Ghost panel + built-in 3D + Raw Messages
  media/                   # screenshot(s) from the harness for the README
```

Also: add `"extensions": "Foxglove Extensions"` to `CATEGORY_NAMES` in `.utils/generate_readme.py`
and regenerate the root `README.md` (`pip install -r .utils/requirements.txt && python
.utils/generate_readme.py` from repo root). Do not commit `node_modules/`, `dist/`, `*.foxe`, or
`demo.mcap` (add to `.gitignore`).

## Extension behaviour

### Data / poses
- Settings pick a **pose topic** (dropdown of topics whose schema is supported). Supported schemas:
  - `foxglove.PoseInFrame` (`timestamp`, `frame_id`, `pose`)
  - `foxglove.PosesInFrame` (use first pose)
  - `foxglove.FrameTransform` and `foxglove.FrameTransforms` (`transforms[]`), filtered by a
    **child frame id** setting (`translation`/`rotation`)
  - `geometry_msgs/PoseStamped`, `geometry_msgs/msg/PoseStamped`
  - `nav_msgs/Odometry`, `nav_msgs/msg/Odometry` (`pose.pose`)
  - `tf2_msgs/TFMessage`, `tf2_msgs/msg/TFMessage` (`transforms[].transform`, `child_frame_id`)
  - `geometry_msgs/TransformStamped` (+ `/msg/`)
  Use the message `receiveTime` (log time) as the timeline key, because that is what the playback
  bar and `currentTime` use. No TF-tree resolution: poses are drawn in the topic's own frame, which
  becomes the panel's fixed/world frame. Document this.
- Loading: when the topic changes, cancel the previous range subscription (the returned unsubscribe
  fn) and start `subscribeMessageRange`. Accumulate extracted poses (not raw messages — keep memory
  small) into `PoseTimeline`; re-render as batches arrive; show a small "loading N poses…" overlay.
  Messages can arrive out of order across batches: insert sorted or sort once per batch.
- `PoseTimeline.sample(tSec, mode)`: binary search; `mode = "interpolate"` (lerp position, slerp
  quaternion, normalize) or `"previous"` (latest pose at or before t). Before the first sample →
  undefined (hide model); after last → clamp to last. Provide `path(): Float32Array` for the trail.

### Rendering (three.js, `GhostScene`)
- WebGLRenderer sized to the panel via ResizeObserver; `OrbitControls` (from
  `three/examples/jsm/controls/OrbitControls.js`); hemisphere + directional light; ground grid
  (toggleable); background follows `renderState.colorScheme`.
- **Current model**: rendered at `sample(currentTime)`. Default model is the procedural haul truck
  (yellow dump body, cab, 6 dark wheels; roughly 9 m × 5 m × 4.5 m, +X forward, Z up — Foxglove/ROS
  convention, so set `camera.up = (0,0,1)`). Optional: **model URL** setting (`.glb`/`.gltf` via
  `GLTFLoader`, also from `three/examples/jsm`) with scale + yaw/pitch/roll offset (glTF is Y-up:
  apply a default +90° X rotation, documented, user-overridable). On load failure show the error in
  the overlay and fall back to the truck.
- **Ghost model**: a deep clone of the current model with ghost materials, rendered at
  `sample(previewTimeSec)` **only when** ghost is enabled, `previewTime` is defined, and the ghost
  pose differs from the current pose. Hidden otherwise. Ghost style settings:
  - style: `wireframe` (default, like the screenshot: `EdgesGeometry`+`LineSegments` overlay plus a
    faint transparent fill so it reads as a holographic model), `transparent`, or `solid`
  - color (default `#4fc3f7`-ish light blue), opacity (0–1, default 0.6)
  - Materials: `depthWrite=false` for transparent parts, `renderOrder` above the current model.
  - Dispose cloned geometries/materials when the model is replaced.
- **Path**: optional line of all poses (default on, color `#2b3cff`-ish blue, as in screenshot). Also
  optionally highlight the segment between current and preview time (brighter / thicker). Use
  `Line2`/`LineMaterial` from three/examples for width > 1 if straightforward; plain `Line` is fine.
- Optional **ghost label**: small HTML overlay or sprite showing `+Δt s` relative to current time.
- **Camera follow** setting: `off` | `current` | `ghost`. `current` keeps the orbit target on the
  current model (preserving user orbit offset). On first data load, frame the whole path.
- Render on demand (only when state/camera changes), not a continuous rAF loop; but do run a rAF when
  OrbitControls are damping/active.

### Panel-driven preview (nice-to-have, implement if cheap)
- Hovering the path in the panel sets `context.setPreviewTime(t)` (raycast to nearest path vertex →
  its timestamp); mouse leave clears it. Click on the path → `context.seekPlayback?.(t)`. This makes
  the ghost previewable from inside the panel too and syncs other panels (Map/Plot) to the same time.
  Must not fight with orbit dragging (only on hover without buttons pressed; seek only on click
  without drag).

### Settings tree (`context.updatePanelSettingsEditor`) and persisted state
- Persist config with `context.saveState`; read `context.initialState` merged over defaults.
- Nodes:
  - `general`: pose topic (select), child frame id (string, shown for TF-like schemas),
    interpolation (`interpolate` | `previous`)
  - `model`: source (`truck` | `url`), url, scale, yaw/pitch/roll offsets (deg), color for truck body
  - `ghost`: **Preview** toggle (`visible` on node — matches the "Preview ON | OFF" ask), style,
    color, opacity, show time label
  - `path`: visible toggle, color, highlight preview segment
  - `view`: follow mode, show grid
- Handle `actionHandler` for `update` (and `perform-node-action` if any) using a pure reducer in
  `settings.ts` (unit-testable).
- Use `context.watch` for `currentTime`, `previewTime`, `topics`, `colorScheme`, `startTime`,
  `endTime`. Always call `done()` at the end of `onRender`.

### Code quality
- TypeScript strict as configured by the scaffold; must pass `npm run build`, `npm run lint`
  (scaffold ESLint config `@foxglove/eslint-plugin`), and `tsc --noEmit`. Use `@foxglove/extension`
  latest 3.x if it builds with the scaffold tooling; otherwise the scaffold default.
- Keep Foxglove-API-specific code in `GhostPreviewPanel.tsx`; scene/pose logic pure and testable.
- No narrating comments. Comments only for non-obvious constraints.

## Demo data (`demo/generate_demo_mcap.py`)
- Uses `foxglove-sdk` Python (`foxglove.open_mcap`, `foxglove.channels`, `foxglove.schemas`).
- ~60 s at 20 Hz, log time = simulated time from a fixed epoch. A truck drives a smooth winding haul
  road (e.g. a spline / sum of sines, a few hundred meters), with heading from the path tangent,
  slight Z from a gentle grade.
- Topics:
  - `/truck/pose` — `foxglove.PoseInFrame`, `frame_id = "map"` (the ghost panel's input)
  - `/tf` — `foxglove.FrameTransforms` map→truck (so the built-in 3D panel can show it too)
  - `/scene/road` — `foxglove.SceneUpdate` once (road ribbon as triangles/line + a few berm cubes) so
    the built-in 3D panel has context
- CLI args: `--output demo.mcap`, `--duration`, `--rate`.

## Testing
1. **Unit tests** (vitest or jest, whichever is simpler with ESM + TS; add `npm test`):
   - `extractPose` for every supported schema, incl. TF child-frame filtering and invalid messages.
   - `PoseTimeline`: ordering with out-of-order inserts, before-first/after-last, exact hits,
     lerp midpoint, slerp normalization and shortest-path (q vs −q), `previous` mode.
   - `time.ts`: previewTime normalization (absolute vs relative).
   - `settings.ts`: reducer updates nested paths, defaults merge with partial saved state.
2. **Build**: `npm run build` and `npm run package` produce a `.foxe`.
3. **Browser harness** (`extension/dev/`, served with Vite or esbuild — dev-only deps): mounts the
   real `initPanel` with a mock `PanelExtensionContext` implementing `watch`, `onRender`,
   `subscribeMessageRange` (feeds synthetic truck poses in batches via an async iterator),
   `saveState`, `updatePanelSettingsEditor` (renders a minimal debug view or logs), `setPreviewTime`,
   `seekPlayback`. Below the panel, a fake timeline bar: click = seek (sets `currentTime`), hover =
   sets `previewTime` (absolute seconds), mouse-leave clears it, plus a play/pause button. Add
   `npm run dev:harness`. Use it to take screenshots (headless Chrome at `/usr/local/bin/google-chrome`
   is available, e.g. via puppeteer-core or `google-chrome --headless --screenshot`) showing: (a) no
   hover → only current truck; (b) hover later in timeline → wireframe ghost further along the path.
   Save to `media/` and reference them from the README.
4. **Demo generator**: run it, then verify with the `mcap` CLI or python `mcap` reader that topics,
   schemas and message counts are as expected.

## README (`extensions/model_ghost_preview/README.md`)
Front-matter: `title: "Model Ghost Preview extension"`, `short_description: "Preview a 3D model as a
ghost at the hovered timeline time, even while paused"`. Sections: what it does (screenshot), why a
custom panel (limitations of converters / built-in 3D), install (`npm ci && npm run local-install`
for desktop, or `npm run package` and drag the `.foxe` into Foxglove), generate demo data, import the
layout, settings reference, supported schemas, limitations (single fixed frame, no TF tree, range
loading is best-effort / memory, preview only on recorded data — live sources have no future data),
development (tests, harness).
