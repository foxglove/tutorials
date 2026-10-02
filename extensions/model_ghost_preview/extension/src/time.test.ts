import { describe, expect, it } from "vitest";

import { normalizePreviewTime, toSec } from "./time";

describe("toSec", () => {
  it("combines seconds and nanoseconds", () => {
    expect(toSec({ sec: 10, nsec: 500_000_000 })).toBeCloseTo(10.5);
    expect(toSec({ sec: 0, nsec: 0 })).toBe(0);
  });
});

describe("normalizePreviewTime", () => {
  const start = { sec: 1_700_000_000, nsec: 0 };

  it("returns undefined when preview time is unset", () => {
    expect(normalizePreviewTime(undefined, start)).toBeUndefined();
  });

  it("keeps absolute preview times that already include the start", () => {
    expect(normalizePreviewTime(1_700_000_010, start)).toBe(1_700_000_010);
  });

  it("adds start time when the preview value looks relative", () => {
    expect(normalizePreviewTime(10, start)).toBe(1_700_000_010);
  });

  it("leaves values within one second before the start unchanged", () => {
    expect(normalizePreviewTime(start.sec - 0.5, start)).toBe(start.sec - 0.5);
  });

  it("does not treat the value as relative when the recording starts near zero", () => {
    expect(normalizePreviewTime(5, { sec: 0, nsec: 0 })).toBe(5);
    expect(normalizePreviewTime(0.25, { sec: 1, nsec: 0 })).toBe(0.25);
  });

  it("returns the raw value when start time is missing", () => {
    expect(normalizePreviewTime(12, undefined)).toBe(12);
  });
});
