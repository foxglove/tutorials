import {
  BoxGeometry,
  CanvasTexture,
  Color,
  CylinderGeometry,
  Group,
  Mesh,
  MeshStandardMaterial,
  SRGBColorSpace,
  type BufferGeometry,
  type Material,
} from "three";

import type { Vec3 } from "../poses/extractPose";

type Part = {
  geometry: BufferGeometry;
  material: Material;
  position: Vec3;
  rotation?: Vec3;
};

const DEFAULT_BODY = "#e6b422";

export function createTruckModel(bodyColor: string): Group {
  const color = cssColor(bodyColor, DEFAULT_BODY);
  const body = new MeshStandardMaterial({
    color: "#ffffff",
    map: paintBody(color),
    roughness: 0.55,
    metalness: 0.12,
  });
  const bed = new MeshStandardMaterial({ color: "#f0c14a", roughness: 0.72, metalness: 0.06 });
  const dark = new MeshStandardMaterial({ color: "#24282c", roughness: 0.55, metalness: 0.35 });
  const tire = new MeshStandardMaterial({ color: "#141414", roughness: 0.92, metalness: 0.02 });
  const hub = new MeshStandardMaterial({ color: "#6a7178", roughness: 0.45, metalness: 0.55 });
  const glass = new MeshStandardMaterial({
    color: "#1b2836",
    roughness: 0.12,
    metalness: 0.7,
  });
  const lamp = new MeshStandardMaterial({
    color: "#ffe7a3",
    emissive: "#ffcc66",
    emissiveIntensity: 0.7,
    roughness: 0.4,
  });

  const root = new Group();
  const wheelGeo = new CylinderGeometry(1.05, 1.05, 0.78, 16);
  const hubGeo = new CylinderGeometry(0.42, 0.42, 0.92, 12);
  const axle = Math.PI / 2;

  add(root, { geometry: box(6.2, 2.05, 0.48), material: dark, position: [0.15, 0, 1.42] });
  add(root, { geometry: box(5.9, 3.55, 0.22), material: body, position: [-1.05, 0, 1.95] });
  add(root, { geometry: box(5.45, 3.2, 0.08), material: bed, position: [-1.05, 0, 2.1] });
  add(root, { geometry: box(5.9, 0.14, 1.75), material: body, position: [-1.05, 1.84, 2.95] });
  add(root, { geometry: box(5.9, 0.14, 1.75), material: body, position: [-1.05, -1.84, 2.95] });
  add(root, { geometry: box(0.16, 3.55, 1.5), material: body, position: [-3.95, 0, 2.8] });
  add(root, { geometry: box(0.18, 3.55, 2.45), material: body, position: [1.85, 0, 3.3] });
  add(root, { geometry: box(5.4, 0.12, 0.1), material: dark, position: [-1.05, 1.84, 3.88] });
  add(root, { geometry: box(5.4, 0.12, 0.1), material: dark, position: [-1.05, -1.84, 3.88] });

  add(root, { geometry: box(1.9, 2.45, 1.95), material: body, position: [2.85, 0, 2.78] });
  add(root, { geometry: box(2.1, 2.7, 0.14), material: body, position: [2.75, 0, 3.82] });
  add(root, { geometry: box(1.2, 2.15, 1.15), material: body, position: [4.1, 0, 2.15] });
  add(root, { geometry: box(0.38, 2.55, 0.58), material: dark, position: [4.72, 0, 1.12] });
  add(root, { geometry: box(0.08, 1.45, 0.72), material: dark, position: [4.9, 0, 1.85] });
  add(root, {
    geometry: box(0.08, 2.05, 0.78),
    material: glass,
    position: [3.62, 0, 3.15],
    rotation: [0, -0.35, 0],
  });
  add(root, { geometry: box(0.72, 0.06, 0.48), material: glass, position: [2.7, 1.26, 3.22] });
  add(root, { geometry: box(0.72, 0.06, 0.48), material: glass, position: [2.7, -1.26, 3.22] });
  add(root, { geometry: box(0.16, 0.4, 0.22), material: lamp, position: [4.88, 0.7, 1.35] });
  add(root, { geometry: box(0.16, 0.4, 0.22), material: lamp, position: [4.88, -0.7, 1.35] });

  add(root, {
    geometry: new CylinderGeometry(0.11, 0.13, 1.55, 10),
    material: dark,
    position: [1.95, 1.2, 3.55],
    rotation: [axle, 0, 0],
  });
  add(root, {
    geometry: new CylinderGeometry(0.38, 0.38, 1.7, 14),
    material: dark,
    position: [0.3, 1.4, 1.55],
    rotation: [0, 0, axle],
  });

  const wheels: Vec3[] = [
    [2.55, 1.7, 1.05],
    [2.55, -1.7, 1.05],
    [-1.45, 1.15, 1.05],
    [-1.45, -1.15, 1.05],
    [-1.45, 2.05, 1.05],
    [-1.45, -2.05, 1.05],
  ];
  for (const position of wheels) {
    add(root, { geometry: wheelGeo, material: tire, position, rotation: [0, 0, axle] });
    add(root, { geometry: hubGeo, material: hub, position, rotation: [0, 0, axle] });
  }

  add(root, { geometry: box(0.07, 0.07, 1.9), material: dark, position: [2.15, -1.4, 1.85] });
  add(root, { geometry: box(0.07, 0.07, 1.9), material: dark, position: [2.58, -1.4, 1.85] });
  for (let step = 0; step < 5; step += 1) {
    add(root, {
      geometry: box(0.5, 0.05, 0.05),
      material: dark,
      position: [2.36, -1.4, 1.15 + step * 0.32],
    });
  }
  add(root, { geometry: box(0.45, 0.7, 0.08), material: dark, position: [3.55, -1.15, 0.7] });
  add(root, { geometry: box(0.45, 0.7, 0.08), material: dark, position: [3.55, -1.15, 1.05] });

  root.traverse((obj) => {
    if (obj instanceof Mesh) {
      obj.castShadow = true;
      obj.receiveShadow = true;
    }
  });
  return root;
}

function add(parent: Group, part: Part): void {
  const mesh = new Mesh(part.geometry, part.material);
  mesh.position.set(part.position[0], part.position[1], part.position[2]);
  if (part.rotation) {
    mesh.rotation.set(part.rotation[0], part.rotation[1], part.rotation[2]);
  }
  parent.add(mesh);
}

function box(width: number, depth: number, height: number): BufferGeometry {
  return new BoxGeometry(width, depth, height);
}

function cssColor(color: string, fallback: string): string {
  try {
    return `#${new Color(color).getHexString()}`;
  } catch {
    return fallback;
  }
}

function paintBody(css: string): CanvasTexture {
  const canvas = document.createElement("canvas");
  canvas.width = 256;
  canvas.height = 256;
  const texture = new CanvasTexture(canvas);
  texture.colorSpace = SRGBColorSpace;
  texture.anisotropy = 8;
  const ctx = canvas.getContext("2d");
  if (!ctx) {
    return texture;
  }
  ctx.fillStyle = css;
  ctx.fillRect(0, 0, 256, 256);
  ctx.fillStyle = "rgba(255,255,255,0.16)";
  ctx.fillRect(0, 0, 256, 48);
  ctx.fillStyle = "rgba(60, 42, 8, 0.28)";
  ctx.fillRect(0, 168, 256, 18);
  ctx.strokeStyle = "rgba(90, 60, 10, 0.35)";
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.moveTo(0, 118);
  ctx.lineTo(256, 118);
  ctx.stroke();
  ctx.fillStyle = "rgba(120, 80, 20, 0.14)";
  for (let i = 0; i < 8; i += 1) {
    ctx.fillRect((i * 53) % 220, 40 + ((i * 37) % 100), 36, 3);
  }
  const fade = ctx.createLinearGradient(0, 0, 0, 256);
  fade.addColorStop(0, "rgba(255,255,255,0.18)");
  fade.addColorStop(0.7, "rgba(255,255,255,0)");
  fade.addColorStop(1, "rgba(90, 60, 0, 0.16)");
  ctx.fillStyle = fade;
  ctx.fillRect(0, 0, 256, 256);
  texture.needsUpdate = true;
  return texture;
}
