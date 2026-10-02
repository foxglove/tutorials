import { Box3, Mesh, Object3D, Vector3 } from "three";
import { describe, expect, it } from "vitest";

import { createTruckModel } from "./truckModel";

describe("createTruckModel", () => {
  it("keeps wheel axles lateral and clear of the chassis and fuel tank", () => {
    const root = createTruckModel("#e6b422");
    root.updateMatrixWorld(true);
    const meshes = collectMeshes(root);
    const wheels = meshes.filter((mesh) => mesh.name === "wheel");
    const chassis = meshes.find((mesh) => mesh.name === "chassis");
    const tank = meshes.find((mesh) => mesh.name === "tank");
    expect(wheels).toHaveLength(6);
    expect(chassis).toBeDefined();
    expect(tank).toBeDefined();
    if (!chassis || !tank) {
      return;
    }
    const chassisBox = new Box3().setFromObject(chassis).expandByScalar(0.02);
    const tankBox = new Box3().setFromObject(tank).expandByScalar(0.02);
    for (const wheel of wheels) {
      expect(wheel.quaternion.x).toBeCloseTo(0, 5);
      expect(wheel.quaternion.y).toBeCloseTo(0, 5);
      expect(wheel.quaternion.z).toBeCloseTo(0, 5);
      const bounds = new Box3().setFromObject(wheel);
      const size = bounds.getSize(new Vector3());
      expect(size.y).toBeLessThan(size.x);
      expect(size.y).toBeLessThan(size.z);
      expect(chassisBox.intersectsBox(bounds)).toBe(false);
      expect(tankBox.intersectsBox(bounds)).toBe(false);
    }
  });
});

function collectMeshes(root: Object3D): Mesh[] {
  const meshes: Mesh[] = [];
  root.traverse((obj) => {
    if (obj instanceof Mesh) {
      meshes.push(obj as Mesh);
    }
  });
  return meshes;
}
