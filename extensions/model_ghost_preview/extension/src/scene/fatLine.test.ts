import { describe, expect, it } from "vitest";

import { lineGeometryForPositions } from "./GhostScene";

describe("lineGeometryForPositions", () => {
  it("reuses the geometry when the vertex count stays the same and replaces it when the count changes", () => {
    const first = lineGeometryForPositions(undefined, vertices(4));
    const sameCount = lineGeometryForPositions(first, vertices(4));
    expect(sameCount).toBe(first);
    const grown = lineGeometryForPositions(first, vertices(12));
    expect(grown).not.toBe(first);
    expect(lineGeometryForPositions(grown, vertices(12))).toBe(grown);
  });
});

function vertices(count: number): Float32Array {
  const positions = new Float32Array(count * 3);
  for (let index = 0; index < count; index += 1) {
    positions[index * 3] = index;
    positions[index * 3 + 1] = index * 0.1;
    positions[index * 3 + 2] = 0;
  }
  return positions;
}
