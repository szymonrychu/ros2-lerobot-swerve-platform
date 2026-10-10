/** Load outcomes of GPS map tiles, summarised into a caption for the layers panel. */

export type TileResult = { ok: true } | { ok: false; status: number | null }
export type TileResults = Record<string, TileResult>

/**
 * Record one tile outcome (later outcomes replace earlier ones).
 *
 * @param results - current outcomes by tile URL
 * @param url - tile URL
 * @param result - outcome; status is the HTTP status of a failed tile when known
 * @returns a new outcome map
 */
export function recordTileResult(results: TileResults, url: string, result: TileResult): TileResults {
  return { ...results, [url]: result }
}

/**
 * Caption describing failed tiles among the tiles currently wanted.
 *
 * @param results - outcomes by tile URL
 * @param urls - tile URLs currently displayed (other outcomes are ignored)
 * @returns null when nothing failed, else a short message
 */
export function tileFailureCaption(results: TileResults, urls: string[]): string | null {
  const settled = urls.map((u) => results[u]).filter((r): r is TileResult => r !== undefined)
  const failed = settled.filter((r): r is { ok: false; status: number | null } => !r.ok)
  if (failed.length === 0) return null
  if (failed.length < settled.length) return `${failed.length} of ${settled.length} map tiles failed to load`
  const statuses = new Set(failed.map((f) => f.status))
  const [only] = [...statuses]
  return statuses.size === 1 && only !== null ? `Map tiles failed to load (HTTP ${only})` : 'Map tiles unavailable'
}
