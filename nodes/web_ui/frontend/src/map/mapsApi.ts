/** Typed calls for the named maps endpoints (GET /api/maps, POST /api/maps/save|load|delete). */
import { useCallback, useEffect, useState } from 'react'
import { parseActionResult } from './mapActions'
import type { ActionResult } from './mapActions'
import type { MapPose, MapStatus } from '../types'

/** Longest wait between status polls while the Map tab is visible. */
export const MAP_STATUS_POLL_MS = 5000
/** Mirror of ros2_common.maps.MAP_NAME_PATTERN and MAP_NAME_RULE. */
export const MAP_NAME_PATTERN = /^[A-Za-z0-9][A-Za-z0-9_-]{0,39}$/
export const MAP_NAME_RULE = "1-40 characters: letters, digits, '_' and '-', starting with a letter or digit"
/** Mirror of ros2_common.maps.MAX_NOTE_CHARS. */
export const MAX_NOTE_CHARS = 500

export type MapOpName = 'save' | 'load' | 'delete'

export interface MapCommandBody {
  name: string
  overwrite?: boolean
  note?: string
  pose?: MapPose
}

/**
 * Client-side check of the map name rule.
 *
 * @param name - candidate map name.
 * @returns the rule message when the name is invalid, otherwise null.
 */
export function mapNameError(name: string): string | null {
  return MAP_NAME_PATTERN.test(name) ? null : `Invalid map name: ${MAP_NAME_RULE}`
}

/**
 * Narrow an unknown payload to a MapStatus.
 *
 * @param body - parsed JSON.
 * @returns the status, or null when the payload has no maps list and localisation.
 */
export function asMapStatus(body: unknown): MapStatus | null {
  const b = body as Partial<MapStatus> | null
  if (!b || !Array.isArray(b.maps) || typeof b.localisation?.state !== 'string') return null
  return b as MapStatus
}

/**
 * GET /api/maps.
 *
 * @param tabId - map_nav tab id.
 * @returns the status, or null when none is available (no status yet, request failed).
 */
export async function fetchMapStatus(tabId: string): Promise<MapStatus | null> {
  try {
    const resp = await fetch(`/api/maps?tab=${encodeURIComponent(tabId)}`)
    if (resp.status !== 200) return null
    return asMapStatus(await resp.json())
  } catch {
    return null
  }
}

/**
 * POST one named map command.
 *
 * @param op - save, load or delete.
 * @param tabId - map_nav tab id.
 * @param body - command fields (name, and overwrite/note for save, pose for load).
 * @returns the displayable result; a failed request becomes an error result.
 */
export async function postMapCommand(op: MapOpName, tabId: string, body: MapCommandBody): Promise<ActionResult> {
  try {
    const resp = await fetch(`/api/maps/${op}?tab=${encodeURIComponent(tabId)}`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body),
    })
    return parseActionResult(resp.status, await resp.json())
  } catch (err) {
    return { state: 'error', message: `Request failed: ${String(err)}` }
  }
}

/**
 * Poll the map status while the page is visible, at most every MAP_STATUS_POLL_MS.
 *
 * @param tabId - map_nav tab id.
 * @returns the latest status (undefined until one arrives) and a function that refreshes it now.
 */
export function useMapStatus(tabId: string): { status: MapStatus | undefined; refresh: () => Promise<void> } {
  const [status, setStatus] = useState<MapStatus | undefined>(undefined)
  const refresh = useCallback(async () => {
    const next = await fetchMapStatus(tabId)
    if (next) setStatus(next)
  }, [tabId])
  useEffect(() => {
    void refresh()
    const timer = setInterval(() => {
      if (document.visibilityState !== 'hidden') void refresh()
    }, MAP_STATUS_POLL_MS)
    return () => clearInterval(timer)
  }, [refresh])
  return { status, refresh }
}
