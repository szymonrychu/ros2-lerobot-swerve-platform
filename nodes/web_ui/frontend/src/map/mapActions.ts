/** Pure helpers for the Map tab action buttons (Stop, Save map, Reset map). */

/** How long the first click on a two-step button stays armed, in milliseconds. */
export const RESET_CONFIRM_MS = 4000

export type ActionState = 'idle' | 'busy' | 'ok' | 'warning' | 'error'

export interface ActionResult {
  state: ActionState
  message: string
}

/**
 * Two-click confirmation: the first click arms, a second click within windowMs fires.
 * @param armedAt time (ms) of the arming click, or null when not armed
 * @param now time (ms) of this click
 * @param windowMs how long an arming click stays valid
 * @returns whether to fire now and the new armedAt (null after firing)
 */
export function confirmClick(
  armedAt: number | null,
  now: number,
  windowMs: number = RESET_CONFIRM_MS,
): { fire: boolean; armedAt: number | null } {
  if (armedAt !== null && now - armedAt <= windowMs) return { fire: true, armedAt: null }
  return { fire: false, armedAt: now }
}

/**
 * True when a topic payload is the backend's explicit "cache cleared" event (data: null),
 * as opposed to undefined (nothing received yet).
 */
export function isCleared(value: unknown): value is null {
  return value === null
}

/**
 * Turn an action endpoint response ({ok, message, map3d?: {status, message}}) into a displayable result.
 * @param status HTTP status code
 * @param body parsed JSON body (any shape)
 */
export function parseActionResult(status: number, body: unknown): ActionResult {
  const b = (body ?? {}) as { ok?: unknown; message?: unknown; map3d?: unknown }
  let message = typeof b.message === 'string' && b.message ? b.message : `HTTP ${status}`
  let state: ActionState = b.ok === true ? 'ok' : 'error'
  const m3 = b.map3d as { status?: unknown; message?: unknown } | null | undefined
  if (m3 && typeof m3 === 'object' && typeof m3.status === 'string') {
    message += `. 3D map: ${m3.status}${typeof m3.message === 'string' && m3.message ? ` - ${m3.message}` : ''}`
    if (state === 'ok' && m3.status !== 'ok') state = 'warning'
  }
  return { state, message }
}

/**
 * POST a tab action route (e.g. /api/arm/home) with the tab id and turn the answer into a displayable result.
 * Never throws: a network failure is an error result.
 *
 * @param path route path without the query string
 * @param tabId id of the tab the route acts for
 * @returns the parsed result
 */
export async function postTabAction(path: string, tabId: string): Promise<ActionResult> {
  try {
    const resp = await fetch(`${path}?tab=${encodeURIComponent(tabId)}`, { method: 'POST' })
    return parseActionResult(resp.status, await resp.json())
  } catch (err) {
    return { state: 'error', message: `Request failed: ${String(err)}` }
  }
}
