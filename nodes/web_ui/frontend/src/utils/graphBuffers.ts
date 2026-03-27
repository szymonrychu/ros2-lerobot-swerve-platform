/**
 * Module-level store for graph time-series buffers.
 *
 * Keyed by tab.id so buffers survive tab unmount/remount across tab switches.
 * Each entry is an array of number arrays: index 0 = timestamps, rest = series values.
 */
const store: Map<string, number[][]> = new Map()

/**
 * Get or create a buffer for the given tab.
 *
 * @param tabId - Unique tab identifier.
 * @param seriesCount - Number of data series (excludes the timestamp array at index 0).
 * @returns number[][] - Array of length seriesCount + 1, index 0 is timestamps.
 */
export function getOrCreateBuffer(tabId: string, seriesCount: number): number[][] {
  const existing = store.get(tabId)
  if (existing && existing.length === seriesCount + 1) return existing
  const fresh: number[][] = Array.from({ length: seriesCount + 1 }, () => [])
  store.set(tabId, fresh)
  return fresh
}

/**
 * Replace the buffer for a tab (e.g. when series count changes due to config reload).
 *
 * @param tabId - Unique tab identifier.
 * @param buf - New buffer to store.
 */
export function setBuffer(tabId: string, buf: number[][]): void {
  store.set(tabId, buf)
}
