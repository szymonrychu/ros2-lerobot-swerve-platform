/** Client of the backend POST /api/poi endpoint (POI edits are not robot motion: allowed during battery cut-off). */
import type { PoiCommand, PoiCreator, PoiKind, PoiResult, PoiStatus } from './types'

/**
 * Send one POI command and wait for poi_store's answer. Never throws.
 *
 * @param tabId - map_nav tab id (selects the topics on the backend)
 * @param command - op and POI fields (the backend adds the request id)
 * @param fetchImpl - fetch implementation (tests)
 * @returns the store result, or an error result for network/HTTP failures
 */
export async function postPoi(tabId: string, command: PoiCommand, fetchImpl: typeof fetch = fetch): Promise<PoiResult> {
  let resp: Response
  try {
    resp = await fetchImpl(`/api/poi?tab=${encodeURIComponent(tabId)}`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(command),
    })
  } catch (err) {
    return { ok: false, message: `Request failed: ${String(err)}`, poi: null }
  }
  try {
    const body = (await resp.json()) as Partial<PoiResult>
    const raw = body.poi as unknown as { removed?: unknown } | null | undefined
    const removed = typeof raw?.removed === 'number' ? raw.removed : undefined
    return {
      ok: body.ok === true,
      message: String(body.message ?? `HTTP ${resp.status}`),
      poi: removed === undefined ? (body.poi ?? null) : null,
      ...(removed === undefined ? {} : { removed }),
    }
  } catch {
    return { ok: false, message: `Unexpected response (HTTP ${resp.status})`, poi: null }
  }
}

/** Filters of the poi_store clear op; every omitted field matches everything (keepZones defaults to false in the store). */
export interface ClearPoisFilters {
  createdBy?: PoiCreator | null
  kinds?: PoiKind[]
  statuses?: PoiStatus[]
  keepZones?: boolean
  ids?: string[]
}

/**
 * Delete the POIs matching the filters (op clear). The single client of the clear route: the Clear POIs dialog and the
 * POI list's "delete marked" (with ids) both call it. Never throws.
 *
 * @param tabId - map_nav tab id
 * @param filters - creator, kinds, statuses, keepZones and ids; only the given fields are sent
 * @param fetchImpl - fetch implementation (tests)
 * @returns the store result; result.removed is the number of deleted POIs
 */
export function clearPois(tabId: string, filters: ClearPoisFilters = {}, fetchImpl: typeof fetch = fetch): Promise<PoiResult> {
  const command: PoiCommand = { op: 'clear' }
  if (filters.createdBy !== undefined) command.created_by = filters.createdBy
  if (filters.kinds !== undefined) command.kinds = filters.kinds
  if (filters.statuses !== undefined) command.statuses = filters.statuses
  if (filters.keepZones !== undefined) command.keep_zones = filters.keepZones
  if (filters.ids !== undefined) command.ids = filters.ids
  return postPoi(tabId, command, fetchImpl)
}
