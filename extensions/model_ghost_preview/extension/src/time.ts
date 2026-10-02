export type TimeLike = {
  sec: number;
  nsec: number;
};

export function toSec(time: TimeLike): number {
  return time.sec + time.nsec / 1e9;
}

/**
 * Foxglove's `previewTime` is absolute seconds (`toSec(startTime) + hoverOffset`).
 * Some callers pass a small offset instead. When `startTime` is far from the epoch
 * and `previewTime` falls more than one second before it, treat the value as relative.
 */
export function normalizePreviewTime(
  previewTime: number | undefined,
  startTime: TimeLike | undefined,
): number | undefined {
  if (previewTime == undefined) {
    return undefined;
  }
  if (startTime == undefined) {
    return previewTime;
  }
  const startSec = toSec(startTime);
  const startIsNonTrivial = Math.abs(startSec) > 1;
  if (startIsNonTrivial && previewTime < startSec - 1) {
    return previewTime + startSec;
  }
  return previewTime;
}
