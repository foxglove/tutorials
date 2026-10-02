import { BoxGeometry, Color, CylinderGeometry, Euler, Group, Mesh, MeshStandardMaterial, Quaternion, Vector3, type BufferGeometry, type Material } from "three";

import type { Vec3 } from "../poses/extractPose";

type Part = {
  name: string;
  geometry: BufferGeometry;
  material: Material;
  position: Vec3;
  rotation?: Vec3;
};

const DEFAULT_BODY = "#e6b422";

export function createTruckModel(bodyColor: string): Group {
  const paint = cssColor(bodyColor, DEFAULT_BODY);
  const body = steel(paint, 0.46, 0.22);
  const bed = steel(shade(paint, 0.82), 0.55, 0.16);
  const dark = steel("#2a2e32", 0.5, 0.4);
  const tire = steel("#141414", 0.92, 0.04);
  const hub = steel("#6d747b", 0.4, 0.62);
  const glass = steel("#1b2836", 0.12, 0.7);
  const lamp = new MeshStandardMaterial({
    color: "#ffe7a3",
    emissive: "#ffcc66",
    emissiveIntensity: 0.7,
    roughness: 0.35,
    metalness: 0.15,
  });

  const root = new Group();
  const wheelGeo = new CylinderGeometry(1.1, 1.1, 0.86, 20);
  const hubGeo = new CylinderGeometry(0.42, 0.42, 0.98, 12);
  const hoistGeo = new CylinderGeometry(0.12, 0.12, 1.15, 12);
  const hoistTilt = tiltTowardBed(0.4);

  add(root, { name: "chassis", geometry: box(7.0, 1.4, 0.42), material: dark, position: [0.3, 0, 1.15] });
  add(root, { name: "frame", geometry: box(6.4, 1.2, 1.22), material: dark, position: [0.15, 0, 1.7] });
  add(root, {
    name: "axle",
    geometry: new CylinderGeometry(0.35, 0.35, 2.76, 16),
    material: dark,
    position: [3.2, 0, 1.1],
  });
  add(root, {
    name: "axle",
    geometry: new CylinderGeometry(0.35, 0.35, 1.76, 16),
    material: dark,
    position: [-1.3, 0, 1.1],
  });
  add(root, { name: "hoist", geometry: hoistGeo, material: dark, position: [-0.15, 0.78, 1.9], rotation: hoistTilt });
  add(root, { name: "hoist", geometry: hoistGeo, material: dark, position: [-0.15, -0.78, 1.9], rotation: hoistTilt });
  add(root, { name: "deck", geometry: box(2.5, 3.3, 0.16), material: dark, position: [3.25, 0, 2.42] });
  add(root, { name: "bed-floor", geometry: box(5.5, 5.6, 0.16), material: bed, position: [-0.55, 0, 2.48] });
  add(root, { name: "bed-wall", geometry: box(5.5, 0.16, 1.35), material: body, position: [-0.55, 2.72, 3.22] });
  add(root, { name: "bed-wall", geometry: box(5.5, 0.16, 1.35), material: body, position: [-0.55, -2.72, 3.22] });
  add(root, { name: "tailgate", geometry: box(0.14, 5.6, 1.25), material: body, position: [-3.25, 0, 3.15] });
  add(root, { name: "headboard", geometry: box(0.18, 5.6, 2.05), material: body, position: [2.16, 0, 3.48] });
  add(root, {
    name: "canopy",
    geometry: box(2.3, 5.6, 0.14),
    material: body,
    position: [3.25, 0, 4.42],
  });
  add(root, { name: "rail", geometry: box(5.2, 0.08, 0.08), material: dark, position: [-0.55, 2.72, 3.95] });
  add(root, { name: "rail", geometry: box(5.2, 0.08, 0.08), material: dark, position: [-0.55, -2.72, 3.95] });

  add(root, { name: "cab", geometry: box(1.45, 1.25, 1.3), material: body, position: [3.15, 0.72, 3.15] });
  add(root, { name: "cab-roof", geometry: box(1.6, 1.4, 0.1), material: body, position: [3.1, 0.72, 3.85] });
  add(root, { name: "hood", geometry: box(1.15, 1.45, 0.55), material: body, position: [4.25, 0.15, 2.75] });
  add(root, { name: "bumper", geometry: box(0.28, 2.4, 0.42), material: dark, position: [4.85, 0, 1.55] });
  add(root, { name: "grill", geometry: box(0.08, 1.3, 0.55), material: dark, position: [4.72, 0.1, 2.15] });
  add(root, {
    name: "windshield",
    geometry: box(0.08, 1.15, 0.7),
    material: glass,
    position: [3.72, 0.72, 3.35],
    rotation: [0, -0.4, 0],
  });
  add(root, { name: "window", geometry: box(0.7, 0.06, 0.42), material: glass, position: [3.05, 1.36, 3.28] });
  add(root, { name: "lamp", geometry: box(0.12, 0.32, 0.18), material: lamp, position: [4.9, 0.7, 1.7] });
  add(root, { name: "lamp", geometry: box(0.12, 0.32, 0.18), material: lamp, position: [4.9, -0.55, 1.7] });

  add(root, {
    name: "exhaust",
    geometry: new CylinderGeometry(0.1, 0.12, 1.35, 10),
    material: dark,
    position: [2.35, 1.35, 3.7],
    rotation: [Math.PI / 2, 0, 0],
  });
  add(root, {
    name: "tank",
    geometry: new CylinderGeometry(0.26, 0.26, 1.0, 14),
    material: dark,
    position: [1.45, -1.05, 1.4],
    rotation: [0, 0, Math.PI / 2],
  });

  const wheels: Vec3[] = [
    [3.2, 1.85, 1.1],
    [3.2, -1.85, 1.1],
    [-1.3, 1.35, 1.1],
    [-1.3, -1.35, 1.1],
    [-1.3, 2.3, 1.1],
    [-1.3, -2.3, 1.1],
  ];
  for (const position of wheels) {
    add(root, { name: "wheel", geometry: wheelGeo, material: tire, position });
    add(root, { name: "hub", geometry: hubGeo, material: hub, position });
  }

  add(root, { name: "ladder", geometry: box(0.06, 0.06, 1.5), material: dark, position: [1.95, -0.95, 1.85] });
  add(root, { name: "ladder", geometry: box(0.06, 0.06, 1.5), material: dark, position: [2.35, -0.95, 1.85] });
  for (let step = 0; step < 4; step += 1) {
    add(root, {
      name: "ladder",
      geometry: box(0.46, 0.05, 0.05),
      material: dark,
      position: [2.15, -0.95, 1.25 + step * 0.32],
    });
  }

  root.traverse((obj) => {
    if (obj instanceof Mesh) {
      obj.castShadow = true;
      obj.receiveShadow = true;
    }
  });
  return root;
}

function tiltTowardBed(radians: number): Vec3 {
  const direction = new Vector3(Math.sin(radians), 0, Math.cos(radians));
  const euler = new Euler().setFromQuaternion(new Quaternion().setFromUnitVectors(new Vector3(0, 1, 0), direction));
  return [euler.x, euler.y, euler.z];
}

function add(parent: Group, part: Part): void {
  const mesh = new Mesh(part.geometry, part.material);
  mesh.name = part.name;
  mesh.position.set(part.position[0], part.position[1], part.position[2]);
  if (part.rotation) {
    mesh.rotation.set(part.rotation[0], part.rotation[1], part.rotation[2]);
  }
  parent.add(mesh);
}

function box(width: number, depth: number, height: number): BufferGeometry {
  return new BoxGeometry(width, depth, height);
}

function steel(color: string, roughness: number, metalness: number): MeshStandardMaterial {
  return new MeshStandardMaterial({ color, roughness, metalness });
}

function shade(color: string, factor: number): string {
  const next = new Color(color);
  next.multiplyScalar(factor);
  return `#${next.getHexString()}`;
}

function cssColor(color: string, fallback: string): string {
  try {
    return `#${new Color(color).getHexString()}`;
  } catch {
    return fallback;
  }
}
