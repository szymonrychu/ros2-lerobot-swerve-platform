/** Client of the read-only GET /api/lookups routes (camera images recorded by mcp_server). */

/** Most lookups the backend returns per request (mcp_server list_lookups limit). */
export const LOOKUP_LIMIT = 10

/** Longest side of a list thumbnail the backend renders (2x the 72 px CSS width, for high-DPI screens). */
export const THUMB_PX = 144

/** One recorded camera image (camelCase of the backend metadata). */
export interface Lookup {
  id: string
  /** Unix seconds. */
  ts: number
  /** ISO time of ts. */
  time: string
  camera: string
  tool: string
  note: string
  poiIds: string[]
  mcpSession: string | null
}

/**
 * URL of the lookups list.
 *
 * @param poiId - only lookups linked to this POI; null for all
 * @param limit - most lookups to return
 * @returns the request URL
 */
export function lookupsUrl(poiId: string | null, limit: number): string {
  const poi = poiId === null ? '' : `&poi_id=${encodeURIComponent(poiId)}`
  return `/api/lookups?limit=${limit}${poi}`
}

/**
 * URL of a lookup's JPEG.
 *
 * @param id - lookup id
 * @returns the image URL
 */
export function lookupImageUrl(id: string): string {
  return `/api/lookups/${encodeURIComponent(id)}.jpg`
}

/**
 * URL of a lookup's server-side downscaled JPEG.
 *
 * @param id - lookup id
 * @param maxPx - longest side in pixels
 * @returns the thumbnail URL
 */
export function lookupThumbUrl(id: string, maxPx: number = THUMB_PX): string {
  return `${lookupImageUrl(id)}?max_px=${maxPx}`
}

function parseLookup(raw: unknown): Lookup | null {
  if (typeof raw !== 'object' || raw === null) return null
  const r = raw as Record<string, unknown>
  if (typeof r.id !== 'string') return null
  return {
    id: r.id,
    ts: typeof r.ts === 'number' ? r.ts : 0,
    time: typeof r.time === 'string' ? r.time : '',
    camera: typeof r.camera === 'string' ? r.camera : '',
    tool: typeof r.tool === 'string' ? r.tool : '',
    note: typeof r.note === 'string' ? r.note : '',
    poiIds: Array.isArray(r.poi_ids) ? r.poi_ids.filter((p): p is string => typeof p === 'string') : [],
    mcpSession: typeof r.mcp_session === 'string' ? r.mcp_session : null,
  }
}

/**
 * Fetch lookups, newest first.
 *
 * @param poiId - only lookups linked to this POI; null for all
 * @param limit - most lookups to return
 * @param fetchImpl - fetch implementation (tests)
 * @returns the lookups (an empty list means there are none)
 * @throws Error with a readable reason on a network failure, an HTTP error or an unexpected body
 */
export async function fetchLookups(poiId: string | null, limit: number, fetchImpl: typeof fetch = fetch): Promise<Lookup[]> {
  const resp = await fetchImpl(lookupsUrl(poiId, limit))
  if (!resp.ok) throw new Error(`HTTP ${resp.status}`)
  const body = (await resp.json()) as { lookups?: unknown }
  if (!Array.isArray(body.lookups)) throw new Error('unexpected response')
  return body.lookups.map(parseLookup).filter((l): l is Lookup => l !== null)
}
