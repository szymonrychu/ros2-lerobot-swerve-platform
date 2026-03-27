/**
 * Module-level store for graph time-series buffers.
 *
 * Keyed by tab.id so buffers survive tab unmount/remount across tab switches.
 * Each entry is an array of number arrays: index 0 = timestamps, rest = series values.
 *
 * Data is fed into buffers from App-level regardless of which tab is mounted,
 * so opening a graph tab always shows the full rolling window of data.
 */
import { TabConfig } from '../types'
import { extractField } from './fieldExtract'

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
 * Feed an incoming topicData snapshot into all graph-type tabs' buffers.
 * Called from App-level on every WebSocket update, regardless of active tab.
 *
 * @param tabs - All configured tabs.
 * @param topicData - Latest message map from useRosBridge.
 */
export function feedAllGraphBuffers(
  tabs: TabConfig[],
  topicData: Record<string, unknown>,
): void {
  for (const tab of tabs) {
    if (tab.type !== 'sensor_graph' && tab.type !== 'effector_graph') continue
    const topics = tab.topics ?? []
    const seriesCount = topics.reduce((n, ts) => n + ts.fields.length, 0)
    if (seriesCount === 0) continue

    // Check whether any series has new data
    let hasUpdate = false
    for (const ts of topics) {
      const msgData = topicData[ts.topic] as Record<string, unknown> | undefined
      if (!msgData) continue
      for (const f of ts.fields) {
        if (extractField(msgData, f.path) !== null) { hasUpdate = true; break }
      }
      if (hasUpdate) break
    }
    if (!hasUpdate) continue

    const buf = getOrCreateBuffer(tab.id, seriesCount)
    const now = performance.now()
    buf[0].push(now)
    let seriesIdx = 1
    for (const ts of topics) {
      const msgData = topicData[ts.topic] as Record<string, unknown> | undefined
      for (const f of ts.fields) {
        const val = msgData ? extractField(msgData, f.path) : null
        buf[seriesIdx].push(val ?? NaN)
        seriesIdx++
      }
    }

    // Trim by time window
    const windowMs = (tab.window_s ?? 10) * 1000
    const maxPoints = tab.max_points ?? 500
    const cutoff = now - windowMs
    const firstKeep = buf[0].findIndex((t) => t >= cutoff)
    if (firstKeep > 0) {
      for (let i = 0; i < buf.length; i++) buf[i] = buf[i].slice(firstKeep)
    }
    if (buf[0].length > maxPoints) {
      const trim = buf[0].length - maxPoints
      for (let i = 0; i < buf.length; i++) buf[i] = buf[i].slice(trim)
    }
  }
}
