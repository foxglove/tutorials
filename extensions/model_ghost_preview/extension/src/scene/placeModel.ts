import type { Object3D } from "three";
import { Group } from "three";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";

export type ModelPlacement = {
  scale: number;
  yawDeg: number;
  pitchDeg: number;
  rollDeg: number;
  /** glTF assets are Y-up. "apply" rotates +90° about X before the user offsets. */
  yUpCorrection: "apply" | "skip";
};

export function placeModel(contents: Object3D, placement: ModelPlacement): Group {
  const root = new Group();
  const scale = placement.scale > 0 ? placement.scale : 1;
  root.scale.setScalar(scale);
  root.rotation.set(
    (placement.rollDeg * Math.PI) / 180,
    (placement.pitchDeg * Math.PI) / 180,
    (placement.yawDeg * Math.PI) / 180,
    "ZYX",
  );
  if (placement.yUpCorrection === "apply") {
    const correction = new Group();
    correction.rotation.x = Math.PI / 2;
    correction.add(contents);
    root.add(correction);
  } else {
    root.add(contents);
  }
  return root;
}

export async function loadGltfScene(url: string): Promise<Group> {
  if (url.trim().length === 0) {
    throw new Error("Model URL is empty");
  }
  const loader = new GLTFLoader();
  const gltf = await loader.loadAsync(url);
  return gltf.scene;
}
