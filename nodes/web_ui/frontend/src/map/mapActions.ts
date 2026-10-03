/** Pure helpers for the Map tab action buttons (Stop, Save map, Reset map). */

/** How long the first click on a two-step button stays armed, in milliseconds. */
export const RESET_CONFIRM_MS = 4000

export type ActionState = 'idle' | 'busy' | 'ok' | 'error'

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
 * Turn an action endpoint response ({ok, message}) into a displayable result.
 * @param status HTTP status code
 * @param body parsed JSON body (any shape)
 */
export function parseActionResult(status: number, body: unknown): ActionResult {
  const b = (body ?? {}) as { ok?: unknown; message?: unknown }
  const message = typeof b.message === 'string' && b.message ? b.message : `HTTP ${status}`
  return { state: b.ok === true ? 'ok' : 'error', message }
}
