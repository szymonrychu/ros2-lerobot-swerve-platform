/** Client of the backend POST /api/arm/{jog,take_control,release,home} endpoints. */

export type ArmAction = 'jog' | 'take_control' | 'release' | 'home'

export interface JogSpec {
  kind: 'cartesian' | 'shoulder_pan' | 'wrist_pitch' | 'wrist_roll' | 'gripper'
  direction: 'up' | 'down' | 'forward' | 'back' | 'left' | 'right' | 'open' | 'close'
  step?: number
}

export interface ArmResult {
  ok: boolean
  message: string
}

/**
 * Post one arm command and return the backend's answer. Never throws.
 *
 * @param action - route name under /api/arm
 * @param tabId - arm tab id (selects the tab on the backend)
 * @param jog - jog spec, sent as the body of a `jog` action
 * @param fetchImpl - fetch implementation (tests)
 * @returns the backend ok/message, or an error result for network/HTTP failures
 */
export async function postArm(action: ArmAction, tabId: string, jog?: JogSpec, fetchImpl: typeof fetch = fetch): Promise<ArmResult> {
  let resp: Response
  try {
    resp = await fetchImpl(`/api/arm/${action}?tab=${encodeURIComponent(tabId)}`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: jog ? JSON.stringify(jog) : undefined,
    })
  } catch (err) {
    return { ok: false, message: `Request failed: ${String(err)}` }
  }
  try {
    const body = (await resp.json()) as Partial<ArmResult>
    return { ok: body.ok === true, message: String(body.message ?? `HTTP ${resp.status}`) }
  } catch {
    return { ok: false, message: `Unexpected response (HTTP ${resp.status})` }
  }
}
