/** Client of the backend POST /api/poi endpoint (POI edits are not robot motion: allowed during battery cut-off). */
import type { PoiCommand, PoiResult } from './types'

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
    return { ok: body.ok === true, message: String(body.message ?? `HTTP ${resp.status}`), poi: body.poi ?? null }
  } catch {
    return { ok: false, message: `Unexpected response (HTTP ${resp.status})`, poi: null }
  }
}
