/** Client of the backend POST /api/grasp and /api/grasp/stop endpoints. Never throws. */
import { GraspAnswer, GraspRequest, parseGraspResponse } from './grasp'

async function send(url: string, init: RequestInit, fetchImpl: typeof fetch): Promise<GraspAnswer> {
  let resp: Response
  try {
    resp = await fetchImpl(url, init)
  } catch (err) {
    return parseGraspResponse({ ok: false, message: `Request failed: ${String(err)}` }, 0)
  }
  let body: unknown = null
  try {
    body = await resp.json()
  } catch {
    // not JSON: parseGraspResponse reports the HTTP status
  }
  return parseGraspResponse(body, resp.status)
}

/**
 * Send a plan / execute / release request.
 *
 * plan resolves with the plan answer (the backend waits for it); execute and release resolve with the accepted reply
 * (or an immediate rejection), their progress and result stream on /web_ui/grasp_result.
 *
 * @param tabId - map_nav tab id
 * @param request - contract request
 * @param fetchImpl - fetch implementation (tests)
 */
export function postGrasp(tabId: string, request: GraspRequest, fetchImpl: typeof fetch = fetch): Promise<GraspAnswer> {
  return send(
    `/api/grasp?tab=${encodeURIComponent(tabId)}`,
    { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(request) },
    fetchImpl,
  )
}

/** Stop a running grasp (never blocked by the battery cut-off). */
export function postGraspStop(tabId: string, fetchImpl: typeof fetch = fetch): Promise<GraspAnswer> {
  return send(`/api/grasp/stop?tab=${encodeURIComponent(tabId)}`, { method: 'POST' }, fetchImpl)
}
