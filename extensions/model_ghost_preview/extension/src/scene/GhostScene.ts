import {
  ACESFilmicToneMapping,
  Box3,
  BufferGeometry,
  Color,
  DirectionalLight,
  GridHelper,
  HemisphereLight,
  Line,
  Material,
  Mesh,
  MeshStandardMaterial,
  Object3D,
  PerspectiveCamera,
  PlaneGeometry,
  Points,
  Scene,
  SRGBColorSpace,
  Texture,
  Vector2,
  Vector3,
  WebGLRenderer,
} from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { Line2 } from "three/examples/jsm/lines/Line2.js";
import { LineGeometry } from "three/examples/jsm/lines/LineGeometry.js";
import { LineMaterial } from "three/examples/jsm/lines/LineMaterial.js";

import type { ReceiveTime } from "../poses/PoseTimeline";
import type { Quat, Vec3 } from "../poses/extractPose";
import type { FollowMode } from "../settings";
import { buildGhostObject, type GhostAppearance, type GhostObject } from "./ghostMaterial";

export type ScenePose = {
  position: Vec3;
  orientation: Quat;
};

export type GhostSceneHandlers = {
  onPreviewTime: (timeSec: number | undefined) => void;
  onSeek: (time: ReceiveTime) => void;
};

type FatLine = {
  line: Line2;
  geometry: LineGeometry;
  material: LineMaterial;
};

export class GhostScene {
  #container: HTMLElement;
  #handlers: GhostSceneHandlers;
  #renderer: WebGLRenderer;
  #scene: Scene;
  #camera: PerspectiveCamera;
  #controls: OrbitControls;
  #resizeObserver: ResizeObserver;
  #hemi: HemisphereLight;
  #sun: DirectionalLight;
  #ground: Mesh;
  #groundMaterial: MeshStandardMaterial;
  #grid: GridHelper | undefined;
  #currentRoot = new Object3D();
  #ghostRoot = new Object3D();
  #model: Object3D | undefined;
  #ghost: GhostObject | undefined;
  #appearance: GhostAppearance = { style: "wireframe", color: "#4fc3f7", opacity: 0.6 };
  #pathLine: FatLine | undefined;
  #highlightLine: FatLine | undefined;
  #pathPoints: Vector3[] = [];
  #pathTimes: number[] = [];
  #receiveTimes: ReceiveTime[] = [];
  #pathKey = "";
  #anchor = new Vector3();
  #gridSize = 180;
  #label: HTMLDivElement;
  #labelText = "";
  #labelAnchor = new Vector3();
  #labelActive = false;
  #labelZOffset = 4.6;
  #highlightKey = "";
  #scheme: "dark" | "light" = "dark";
  #gridVisibility: "shown" | "hidden" = "shown";
  #follow: FollowMode = "off";
  #followOffset = new Vector3(-18, -24, 14);
  #currentPose: ScenePose | undefined;
  #ghostPose: ScenePose | undefined;
  #ghostVisible = false;
  #width = 1;
  #height = 1;
  #raf = 0;
  #dirty = true;
  #interacting = false;
  #hoverTime: number | undefined;
  #downX = 0;
  #downY = 0;
  #downButton = -1;
  #scratch = new Vector3();
  #scratchB = new Vector3();
  #forward = new Vector3();

  constructor(container: HTMLElement, handlers: GhostSceneHandlers) {
    this.#container = container;
    this.#handlers = handlers;
    this.#renderer = new WebGLRenderer({ antialias: true, alpha: false });
    this.#renderer.outputColorSpace = SRGBColorSpace;
    this.#renderer.toneMapping = ACESFilmicToneMapping;
    this.#renderer.toneMappingExposure = 1.08;
    this.#renderer.domElement.style.width = "100%";
    this.#renderer.domElement.style.height = "100%";
    this.#renderer.domElement.style.display = "block";
    this.#renderer.domElement.style.touchAction = "none";
    container.appendChild(this.#renderer.domElement);
    this.#label = document.createElement("div");
    this.#label.style.position = "absolute";
    this.#label.style.transform = "translate(-50%, -130%)";
    this.#label.style.padding = "2px 6px";
    this.#label.style.borderRadius = "4px";
    this.#label.style.fontSize = "12px";
    this.#label.style.fontFamily = "system-ui, sans-serif";
    this.#label.style.pointerEvents = "none";
    this.#label.style.whiteSpace = "nowrap";
    this.#label.style.display = "none";
    this.#styleLabel();
    container.appendChild(this.#label);

    this.#scene = new Scene();
    this.#camera = new PerspectiveCamera(42, 1, 0.2, 4000);
    this.#camera.up.set(0, 0, 1);
    this.#camera.position.set(-22, -28, 16);

    this.#controls = new OrbitControls(this.#camera, this.#renderer.domElement);
    this.#controls.enableDamping = true;
    this.#controls.dampingFactor = 0.12;
    this.#controls.target.set(12, 0, 1);
    this.#controls.update();
    this.#followOffset.copy(this.#camera.position).sub(this.#controls.target);
    this.#controls.addEventListener("start", this.#onControlStart);
    this.#controls.addEventListener("end", this.#onControlEnd);
    this.#controls.addEventListener("change", this.#onControlChange);

    this.#hemi = new HemisphereLight(0xf4f7fb, 0x8a8178, 0.9);
    this.#sun = new DirectionalLight(0xfff4e0, 1.25);
    this.#sun.position.set(36, -48, 72);
    this.#scene.add(this.#hemi);
    this.#scene.add(this.#sun);
    this.#scene.add(this.#sun.target);

    this.#groundMaterial = new MeshStandardMaterial({ color: 0xc5ced6, roughness: 1, metalness: 0 });
    this.#ground = new Mesh(new PlaneGeometry(400, 400), this.#groundMaterial);
    this.#ground.receiveShadow = true;
    this.#scene.add(this.#ground);
    this.#scene.add(this.#currentRoot);
    this.#scene.add(this.#ghostRoot);
    this.#ghostRoot.visible = false;

    this.#applyScheme();
    this.#resizeObserver = new ResizeObserver(() => {
      this.#resize();
    });
    this.#resizeObserver.observe(container);
    this.#resize();

    const canvas = this.#renderer.domElement;
    canvas.addEventListener("pointermove", this.#onPointerMove);
    canvas.addEventListener("pointerdown", this.#onPointerDown);
    canvas.addEventListener("pointerup", this.#onPointerUp);
    canvas.addEventListener("pointerleave", this.#onPointerLeave);
  }

  dispose(): void {
    cancelAnimationFrame(this.#raf);
    this.#raf = 0;
    this.#resizeObserver.disconnect();
    const canvas = this.#renderer.domElement;
    canvas.removeEventListener("pointermove", this.#onPointerMove);
    canvas.removeEventListener("pointerdown", this.#onPointerDown);
    canvas.removeEventListener("pointerup", this.#onPointerUp);
    canvas.removeEventListener("pointerleave", this.#onPointerLeave);
    this.#controls.removeEventListener("start", this.#onControlStart);
    this.#controls.removeEventListener("end", this.#onControlEnd);
    this.#controls.removeEventListener("change", this.#onControlChange);
    this.#controls.dispose();
    this.#clearGhost();
    if (this.#model) {
      disposeObjectTree(this.#model);
    }
    this.#removeFatLine(this.#pathLine);
    this.#removeFatLine(this.#highlightLine);
    this.#dropGrid();
    this.#ground.geometry.dispose();
    this.#groundMaterial.dispose();
    this.#renderer.dispose();
    this.#label.remove();
    canvas.remove();
  }

  setColorScheme(scheme: "dark" | "light"): void {
    if (this.#scheme === scheme) {
      return;
    }
    this.#scheme = scheme;
    this.#applyScheme();
    this.#requestRender();
  }

  setGridVisibility(visibility: "shown" | "hidden"): void {
    if (this.#gridVisibility === visibility) {
      return;
    }
    this.#gridVisibility = visibility;
    this.#rebuildGrid();
    this.#requestRender();
  }

  setFollowMode(mode: FollowMode): void {
    this.#follow = mode;
    this.#applyFollow();
    this.#requestRender();
  }

  setModel(model: Object3D): void {
    this.#clearGhost();
    if (this.#model) {
      this.#currentRoot.remove(this.#model);
      disposeObjectTree(this.#model);
    }
    this.#model = model;
    this.#currentRoot.add(model);
    this.#measureLabelOffset();
    this.#rebuildGhost();
  }

  setTimeLabel(label: { text: string; position: Vec3 } | undefined): void {
    if (!label) {
      this.#labelActive = false;
      this.#syncLabel();
      return;
    }
    this.#labelActive = true;
    this.#labelText = label.text;
    this.#labelAnchor.set(label.position[0], label.position[1], label.position[2]);
    this.#syncLabel();
  }

  setGhostAppearance(appearance: GhostAppearance): void {
    if (
      this.#appearance.style === appearance.style &&
      this.#appearance.color === appearance.color &&
      this.#appearance.opacity === appearance.opacity
    ) {
      return;
    }
    this.#appearance = appearance;
    this.#rebuildGhost();
  }

  setCurrentPose(pose: ScenePose | undefined): void {
    this.#currentPose = pose;
    this.#currentRoot.visible = pose != undefined;
    if (pose) {
      applyPose(this.#currentRoot, pose);
      this.#anchor.z = pose.position[2];
      if (this.#grid) {
        this.#grid.position.z = pose.position[2] + 0.02;
      }
    }
    this.#applyFollow();
    this.#requestRender();
  }

  setGhostPose(pose: ScenePose | undefined): void {
    this.#ghostPose = pose;
    this.#ghostVisible = pose != undefined;
    this.#ghostRoot.visible = this.#ghostVisible;
    if (pose) {
      applyPose(this.#ghostRoot, pose);
    }
    this.#applyFollow();
    this.#requestRender();
  }

  setPath(path: {
    points: Float32Array;
    times: Float64Array;
    receiveTimes: readonly ReceiveTime[];
    color: string;
    visibility: "shown" | "hidden";
  }): void {
    const count = Math.min(path.times.length, Math.floor(path.points.length / 3));
    const first = path.points[0] ?? 0;
    const last = path.points[Math.max(0, path.points.length - 1)] ?? 0;
    const key = `${path.visibility}:${path.color}:${count}:${first}:${last}`;
    if (key === this.#pathKey) {
      return;
    }
    this.#pathKey = key;
    this.#pathPoints = [];
    this.#pathTimes = [];
    this.#receiveTimes = [];
    if (path.visibility === "shown") {
      for (let index = 0; index < count; index += 1) {
        const time = path.times[index];
        const receiveTime = path.receiveTimes[index];
        if (time == undefined || !receiveTime) {
          continue;
        }
        this.#pathPoints.push(
          new Vector3(
            path.points[index * 3] ?? 0,
            path.points[index * 3 + 1] ?? 0,
            path.points[index * 3 + 2] ?? 0,
          ),
        );
        this.#pathTimes.push(time);
        this.#receiveTimes.push(receiveTime);
      }
    }
    this.#fitGround(this.#pathPoints);
    this.#pathLine = this.#syncFatLine(this.#pathLine, this.#pathPoints, path.color, 8, 0.15);
    this.#requestRender();
  }

  setHighlight(highlight: { points: Float32Array; color: string } | undefined): void {
    if (!highlight || highlight.points.length < 6) {
      if (this.#highlightKey === "") {
        return;
      }
      this.#highlightKey = "";
      this.#highlightLine = this.#syncFatLine(this.#highlightLine, [], highlight?.color ?? "#ffffff", 11, 0.22);
      this.#requestRender();
      return;
    }
    const first = highlight.points[0] ?? 0;
    const last = highlight.points[highlight.points.length - 1] ?? 0;
    const key = `${highlight.color}:${highlight.points.length}:${first}:${last}`;
    if (key === this.#highlightKey) {
      return;
    }
    this.#highlightKey = key;
    const points: Vector3[] = [];
    for (let index = 0; index < highlight.points.length; index += 3) {
      points.push(
        new Vector3(
          highlight.points[index] ?? 0,
          highlight.points[index + 1] ?? 0,
          highlight.points[index + 2] ?? 0,
        ),
      );
    }
    const color = lighten(highlight.color);
    this.#highlightLine = this.#syncFatLine(this.#highlightLine, points, color, 11, 0.22);
    this.#requestRender();
  }

  framePath(): void {
    const start = this.#pathPoints[0];
    const end = this.#pathPoints[this.#pathPoints.length - 1];
    if (!start || !end) {
      return;
    }
    const bounds = new Box3();
    for (const point of this.#pathPoints) {
      bounds.expandByPoint(point);
    }
    const focus = bounds.getCenter(new Vector3());
    focus.z += 1.2;
    const size = bounds.getSize(new Vector3());
    const radius = Math.max(0.5 * Math.hypot(size.x, size.y, size.z), 12);
    const vertical = (this.#camera.fov * Math.PI) / 180;
    const aspect = this.#width / Math.max(this.#height, 1);
    const horizontal = 2 * Math.atan(Math.tan(vertical / 2) * Math.max(aspect, 0.5));
    const fitAngle = Math.min(vertical, horizontal) / 2;
    const distance = (radius / Math.sin(fitAngle)) * 0.72;
    const direction = new Vector3().subVectors(end, start);
    if (direction.lengthSq() < 1e-6) {
      direction.set(1, 0, 0);
    }
    direction.normalize();
    const side = new Vector3().crossVectors(direction, new Vector3(0, 0, 1));
    if (side.lengthSq() < 1e-6) {
      side.set(0, 1, 0);
    }
    side.normalize();
    this.#camera.position
      .copy(focus)
      .addScaledVector(direction, -distance * 0.42)
      .addScaledVector(side, -distance * 0.72)
      .setZ(focus.z + distance * 0.38);
    this.#controls.target.copy(focus);
    this.#controls.update();
    this.#followOffset.copy(this.#camera.position).sub(this.#controls.target);
    this.#requestRender();
  }

  projectToOverlay(position: Vec3, zOffset: number): { x: number; y: number } | undefined {
    const rect = this.#renderer.domElement.getBoundingClientRect();
    if (rect.width <= 0 || rect.height <= 0) {
      return undefined;
    }
    this.#scratch.set(position[0], position[1], position[2] + zOffset);
    this.#camera.getWorldDirection(this.#forward);
    this.#scratchB.copy(this.#scratch).sub(this.#camera.position);
    if (this.#scratchB.dot(this.#forward) <= 0) {
      return undefined;
    }
    this.#scratch.project(this.#camera);
    return {
      x: (this.#scratch.x * 0.5 + 0.5) * rect.width,
      y: (-this.#scratch.y * 0.5 + 0.5) * rect.height,
    };
  }

  #applyScheme(): void {
    const dark = this.#scheme === "dark";
    this.#scene.background = new Color(dark ? 0x1c1f24 : 0xd5dbe1);
    this.#groundMaterial.color.set(dark ? 0x2a3036 : 0xc5ced6);
    this.#hemi.color.set(dark ? 0x8aa0b8 : 0xf4f7fb);
    this.#hemi.groundColor.set(dark ? 0x3a332c : 0x8a8178);
    this.#hemi.intensity = dark ? 1.35 : 0.92;
    this.#sun.intensity = dark ? 2.15 : 1.35;
    this.#styleLabel();
    this.#rebuildGrid();
  }

  #styleLabel(): void {
    const dark = this.#scheme === "dark";
    this.#label.style.background = dark ? "rgba(236, 246, 252, 0.94)" : "rgba(12, 32, 48, 0.8)";
    this.#label.style.color = dark ? "#102433" : "#d7f3ff";
    this.#label.style.border = dark ? "1px solid #ffffff" : "1px solid transparent";
  }

  #rebuildGrid(): void {
    this.#dropGrid();
    if (this.#gridVisibility !== "shown") {
      return;
    }
    const dark = this.#scheme === "dark";
    const grid = new GridHelper(
      this.#gridSize,
      36,
      dark ? 0x51606c : 0xb7c2cc,
      dark ? 0x3a444e : 0xc9d1d8,
    );
    grid.rotation.x = Math.PI / 2;
    grid.position.set(this.#anchor.x, this.#anchor.y, this.#anchor.z + 0.02);
    this.#scene.add(grid);
    this.#grid = grid;
  }

  #dropGrid(): void {
    if (!this.#grid) {
      return;
    }
    const grid = this.#grid;
    this.#scene.remove(grid);
    grid.geometry.dispose();
    disposeMaterial(grid.material);
    this.#grid = undefined;
  }

  #rebuildGhost(): void {
    this.#clearGhost();
    if (!this.#model) {
      return;
    }
    const ghost = buildGhostObject(this.#model, this.#appearance, {
      width: this.#width,
      height: this.#height,
    });
    this.#ghost = ghost;
    this.#ghostRoot.add(ghost.object);
    this.#ghostRoot.visible = this.#ghostVisible;
    this.#requestRender();
  }

  #clearGhost(): void {
    if (!this.#ghost) {
      return;
    }
    this.#ghostRoot.remove(this.#ghost.object);
    this.#ghost.dispose();
    this.#ghost = undefined;
  }

  #applyFollow(): void {
    if (this.#interacting || this.#follow === "off") {
      return;
    }
    const pose = this.#follow === "current" ? this.#currentPose : this.#ghostPose;
    if (!pose) {
      return;
    }
    this.#controls.target.set(pose.position[0], pose.position[1], pose.position[2] + 1.2);
    this.#camera.position.copy(this.#controls.target).add(this.#followOffset);
    this.#controls.update();
  }

  #fitGround(points: readonly Vector3[]): void {
    if (points.length === 0) {
      return;
    }
    let minX = Infinity;
    let minY = Infinity;
    let minZ = Infinity;
    let maxX = -Infinity;
    let maxY = -Infinity;
    for (const point of points) {
      minX = Math.min(minX, point.x);
      minY = Math.min(minY, point.y);
      minZ = Math.min(minZ, point.z);
      maxX = Math.max(maxX, point.x);
      maxY = Math.max(maxY, point.y);
    }
    const span = Math.max(maxX - minX, maxY - minY, 40);
    this.#gridSize = Math.max(span * 1.5, 80);
    const poseZ = this.#currentPose?.position[2];
    this.#anchor.set((minX + maxX) / 2, (minY + maxY) / 2, poseZ ?? minZ);
    this.#ground.geometry.dispose();
    this.#ground.geometry = new PlaneGeometry(this.#gridSize, this.#gridSize);
    this.#ground.position.set((minX + maxX) / 2, (minY + maxY) / 2, minZ - 0.02);
    this.#rebuildGrid();
  }

  #measureLabelOffset(): void {
    if (!this.#model) {
      this.#labelZOffset = 1;
      return;
    }
    const bounds = new Box3().setFromObject(this.#model);
    const height = bounds.max.z - bounds.min.z;
    this.#labelZOffset = (Number.isFinite(height) && height > 0 ? height : 1) + 0.35;
  }

  #syncLabel(): void {
    if (!this.#labelActive) {
      this.#label.style.display = "none";
      return;
    }
    const point = this.projectToOverlay(
      [this.#labelAnchor.x, this.#labelAnchor.y, this.#labelAnchor.z],
      this.#labelZOffset,
    );
    if (!point) {
      this.#label.style.display = "none";
      return;
    }
    this.#label.style.display = "block";
    this.#label.style.left = `${point.x}px`;
    this.#label.style.top = `${point.y}px`;
    this.#label.textContent = this.#labelText;
  }

  #syncFatLine(
    current: FatLine | undefined,
    points: readonly Vector3[],
    color: string,
    width: number,
    lift: number,
  ): FatLine | undefined {
    if (points.length < 2) {
      this.#removeFatLine(current);
      return undefined;
    }
    const positions = linePositions(points, lift);
    const hex = safeHex(color);
    const geometry = lineGeometryForPositions(current?.geometry, positions);
    if (current?.geometry === geometry) {
      current.material.color.set(hex);
      current.material.linewidth = width;
      return current;
    }
    this.#removeFatLine(current);
    const material = new LineMaterial({
      color: hex,
      linewidth: width,
      transparent: true,
      opacity: 1,
      depthWrite: false,
      depthTest: true,
      resolution: new Vector2(Math.max(this.#width, 1), Math.max(this.#height, 1)),
    });
    const line = new Line2(geometry, material);
    line.frustumCulled = false;
    line.renderOrder = 1;
    this.#scene.add(line);
    return { line, geometry, material };
  }

  #removeFatLine(line: FatLine | undefined): void {
    if (!line) {
      return;
    }
    this.#scene.remove(line.line);
    line.geometry.dispose();
    line.material.dispose();
  }

  #applyResolution(): void {
    const resolution = new Vector2(Math.max(this.#width, 1), Math.max(this.#height, 1));
    this.#pathLine?.material.resolution.copy(resolution);
    this.#highlightLine?.material.resolution.copy(resolution);
    if (this.#ghost) {
      for (const material of this.#ghost.lineMaterials) {
        material.resolution.copy(resolution);
      }
    }
  }

  #resize(): void {
    const width = this.#container.clientWidth;
    const height = this.#container.clientHeight;
    this.#width = width;
    this.#height = height;
    if (width <= 0 || height <= 0) {
      return;
    }
    const ratio = window.devicePixelRatio;
    this.#renderer.setPixelRatio(ratio > 0 ? Math.min(ratio, 2) : 1);
    this.#renderer.setSize(width, height, false);
    this.#camera.aspect = width / height;
    this.#camera.updateProjectionMatrix();
    this.#applyResolution();
    this.#requestRender();
  }

  #requestRender(): void {
    this.#dirty = true;
    if (this.#raf !== 0) {
      return;
    }
    this.#raf = requestAnimationFrame(this.#frame);
  }

  #frame = (): void => {
    this.#raf = 0;
    const moving = this.#controls.update();
    if (this.#dirty || moving) {
      this.#renderer.render(this.#scene, this.#camera);
      this.#syncLabel();
      this.#dirty = false;
    }
    if (moving || this.#interacting) {
      this.#requestRender();
    }
  };

  #onControlStart = (): void => {
    this.#interacting = true;
    this.#requestRender();
  };

  #onControlEnd = (): void => {
    this.#interacting = false;
    this.#followOffset.copy(this.#camera.position).sub(this.#controls.target);
    this.#requestRender();
  };

  #onControlChange = (): void => {
    this.#requestRender();
  };

  #onPointerDown = (event: PointerEvent): void => {
    this.#downX = event.clientX;
    this.#downY = event.clientY;
    this.#downButton = event.button;
  };

  #onPointerUp = (event: PointerEvent): void => {
    const moved = Math.hypot(event.clientX - this.#downX, event.clientY - this.#downY);
    const button = this.#downButton;
    this.#downButton = -1;
    if (event.button !== 0 || button !== 0 || moved > 5) {
      return;
    }
    const index = this.#pickIndex(event.clientX, event.clientY);
    const receiveTime = index == undefined ? undefined : this.#receiveTimes[index];
    if (receiveTime) {
      this.#handlers.onSeek(receiveTime);
    }
  };

  #onPointerMove = (event: PointerEvent): void => {
    if (event.buttons !== 0) {
      return;
    }
    const index = this.#pickIndex(event.clientX, event.clientY);
    this.#emitHover(index == undefined ? undefined : this.#pathTimes[index]);
  };

  #onPointerLeave = (): void => {
    this.#emitHover(undefined);
  };

  #emitHover(time: number | undefined): void {
    if (time === this.#hoverTime) {
      return;
    }
    this.#hoverTime = time;
    this.#handlers.onPreviewTime(time);
  }

  #pickIndex(clientX: number, clientY: number): number | undefined {
    const rect = this.#renderer.domElement.getBoundingClientRect();
    if (rect.width <= 0 || rect.height <= 0 || this.#pathPoints.length === 0) {
      return undefined;
    }
    const x = clientX - rect.left;
    const y = clientY - rect.top;
    this.#camera.getWorldDirection(this.#forward);
    let best = 20;
    let bestIndex: number | undefined;
    for (let index = 0; index < this.#pathPoints.length; index += 1) {
      const point = this.#pathPoints[index];
      if (!point) {
        continue;
      }
      this.#scratch.copy(point).sub(this.#camera.position);
      if (this.#scratch.dot(this.#forward) <= 0) {
        continue;
      }
      this.#scratch.copy(point).project(this.#camera);
      const sx = (this.#scratch.x * 0.5 + 0.5) * rect.width;
      const sy = (-this.#scratch.y * 0.5 + 0.5) * rect.height;
      const distance = Math.hypot(sx - x, sy - y);
      if (distance < best) {
        best = distance;
        bestIndex = index;
      }
    }
    return bestIndex;
  }
}

function applyPose(object: Object3D, pose: ScenePose): void {
  object.position.set(pose.position[0], pose.position[1], pose.position[2]);
  object.quaternion.set(pose.orientation[0], pose.orientation[1], pose.orientation[2], pose.orientation[3]);
}

export function disposeObjectTree(root: Object3D): void {
  const geometries = new Set<BufferGeometry>();
  const materials = new Set<Material>();
  const textures = new Set<Texture>();
  root.traverse((obj) => {
    const drawable = drawableObject(obj);
    if (!drawable) {
      return;
    }
    geometries.add(drawable.geometry);
    collectMaterials(drawable.material, materials);
  });
  for (const geometry of geometries) {
    geometry.dispose();
  }
  for (const material of materials) {
    collectTextures(material, textures);
    material.dispose();
  }
  for (const texture of textures) {
    texture.dispose();
  }
}

function drawableObject(
  obj: Object3D,
): { geometry: BufferGeometry; material: Material | Material[] } | undefined {
  if (obj instanceof Mesh) {
    return obj as Mesh;
  }
  if (obj instanceof Line) {
    return obj as Line;
  }
  if (obj instanceof Points) {
    return obj as Points;
  }
  return undefined;
}

function collectTextures(material: Material, into: Set<Texture>): void {
  const record = material as unknown as Record<string, unknown>;
  for (const key of Object.keys(record)) {
    const value = record[key];
    if (value instanceof Texture) {
      into.add(value);
    }
  }
}

function collectMaterials(material: Material | Material[], into: Set<Material>): void {
  if (isMaterialList(material)) {
    for (const entry of material) {
      into.add(entry);
    }
    return;
  }
  into.add(material);
}

function disposeMaterial(material: Material | readonly Material[]): void {
  if (isMaterialList(material)) {
    for (const entry of material) {
      entry.dispose();
    }
    return;
  }
  material.dispose();
}

function isMaterialList(value: Material | readonly Material[]): value is readonly Material[] {
  return Object.prototype.toString.call(value) === "[object Array]";
}

function linePositions(points: readonly Vector3[], lift: number): Float32Array {
  const positions = new Float32Array(points.length * 3);
  for (let index = 0; index < points.length; index += 1) {
    const point = points[index];
    if (!point) {
      continue;
    }
    positions[index * 3] = point.x;
    positions[index * 3 + 1] = point.y;
    positions[index * 3 + 2] = point.z + lift;
  }
  return positions;
}

export function lineGeometryForPositions(
  current: LineGeometry | undefined,
  positions: Float32Array,
): LineGeometry {
  if (current && writeLinePositions(current, positions)) {
    return current;
  }
  const created = new LineGeometry();
  created.setPositions(positions);
  return created;
}

function writeLinePositions(geometry: LineGeometry, positions: Float32Array): boolean {
  const attribute = geometry.getAttribute("instanceStart");
  if (positions.length < 6) {
    return false;
  }
  const segments = positions.length / 3 - 1;
  if (attribute.count !== segments) {
    return false;
  }
  const array = attribute.array;
  if (!(array instanceof Float32Array) || array.length < segments * 6) {
    return false;
  }
  let write = 0;
  for (let index = 0; index < positions.length - 3; index += 3) {
    array[write] = positions[index] ?? 0;
    array[write + 1] = positions[index + 1] ?? 0;
    array[write + 2] = positions[index + 2] ?? 0;
    array[write + 3] = positions[index + 3] ?? 0;
    array[write + 4] = positions[index + 4] ?? 0;
    array[write + 5] = positions[index + 5] ?? 0;
    write += 6;
  }
  attribute.needsUpdate = true;
  geometry.computeBoundingBox();
  geometry.computeBoundingSphere();
  return true;
}

function lighten(color: string): string {
  try {
    const next = new Color(color);
    next.lerp(new Color("#ffffff"), 0.5);
    return `#${next.getHexString()}`;
  } catch {
    return "#9ecbff";
  }
}

function safeHex(color: string): number {
  try {
    return new Color(color).getHex();
  } catch {
    return 0x2b3cff;
  }
}
