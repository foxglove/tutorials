import {
  BufferGeometry,
  Color,
  DoubleSide,
  EdgesGeometry,
  FrontSide,
  Material,
  Mesh,
  MeshBasicMaterial,
  MeshStandardMaterial,
  Object3D,
  Vector2,
} from "three";
import { LineMaterial } from "three/examples/jsm/lines/LineMaterial.js";
import { LineSegments2 } from "three/examples/jsm/lines/LineSegments2.js";
import { LineSegmentsGeometry } from "three/examples/jsm/lines/LineSegmentsGeometry.js";

import type { GhostStyle } from "../settings";

export type GhostAppearance = {
  style: GhostStyle;
  color: string;
  opacity: number;
};

export type GhostObject = {
  object: Object3D;
  lineMaterials: LineMaterial[];
  dispose: () => void;
};

export function buildGhostObject(
  source: Object3D,
  appearance: GhostAppearance,
  resolution: { width: number; height: number },
): GhostObject {
  const clone = source.clone(true);
  const materials: Material[] = [];
  const geometries: BufferGeometry[] = [];
  const lineMaterials: LineMaterial[] = [];
  const color = safeColor(appearance.color);
  const opacity = clamp(appearance.opacity, 0, 1);
  const width = Math.max(resolution.width, 1);
  const height = Math.max(resolution.height, 1);
  const meshes: Mesh[] = [];
  clone.traverse((obj) => {
    const mesh = concreteMesh(obj);
    if (mesh) {
      meshes.push(mesh);
    }
  });

  for (const mesh of meshes) {
    const position = mesh.geometry.getAttribute("position");
    if (position.count === 0) {
      continue;
    }
    const fill = fillMaterial(appearance.style, color, opacity);
    mesh.material = fill;
    mesh.renderOrder = 2;
    mesh.castShadow = false;
    materials.push(fill);
    if (appearance.style !== "wireframe") {
      continue;
    }
    const edges = new EdgesGeometry(mesh.geometry, 20);
    const edgePosition = edges.getAttribute("position");
    if (edgePosition.count < 2) {
      edges.dispose();
      continue;
    }
    const positions = new Float32Array(edgePosition.array.length);
    for (let index = 0; index < edgePosition.array.length; index += 1) {
      positions[index] = edgePosition.array[index] ?? 0;
    }
    edges.dispose();
    const segments = new LineSegmentsGeometry();
    segments.setPositions(positions);
    const lineMaterial = new LineMaterial({
      color: color.getHex(),
      linewidth: 0.12,
      worldUnits: true,
      transparent: true,
      opacity: Math.min(1, opacity + 0.45),
      depthWrite: false,
      depthTest: true,
      resolution: new Vector2(width, height),
    });
    const lines = new LineSegments2(segments, lineMaterial);
    lines.renderOrder = 3;
    lines.computeLineDistances();
    mesh.add(lines);
    geometries.push(segments);
    materials.push(lineMaterial);
    lineMaterials.push(lineMaterial);
  }

  return {
    object: clone,
    lineMaterials,
    dispose: () => {
      for (const geometry of geometries) {
        geometry.dispose();
      }
      for (const material of materials) {
        material.dispose();
      }
    },
  };
}

function fillMaterial(style: GhostStyle, color: Color, opacity: number): Material {
  if (style === "solid") {
    const transparent = opacity < 0.999;
    return new MeshStandardMaterial({
      color,
      transparent,
      opacity,
      depthWrite: !transparent,
      depthTest: true,
      roughness: 0.45,
      metalness: 0.1,
      side: FrontSide,
    });
  }
  const fillOpacity = style === "wireframe" ? Math.min(0.16, opacity * 0.22) : opacity;
  return new MeshBasicMaterial({
    color,
    transparent: true,
    opacity: fillOpacity,
    depthWrite: false,
    depthTest: true,
    side: DoubleSide,
  });
}

function safeColor(color: string): Color {
  try {
    return new Color(color);
  } catch {
    return new Color("#4fc3f7");
  }
}

function clamp(value: number, min: number, max: number): number {
  return Math.min(max, Math.max(min, value));
}

function concreteMesh(obj: Object3D): Mesh | undefined {
  if (!(obj instanceof Mesh)) {
    return undefined;
  }
  return obj as Mesh;
}
