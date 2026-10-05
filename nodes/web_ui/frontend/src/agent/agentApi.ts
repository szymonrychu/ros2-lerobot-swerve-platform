/** Calls of the backend's claude_agent proxy (/api/agent/*). */
import type { AgentInfo } from './agentModel'

export interface AgentActionResult {
  ok: boolean
  status: number
  message: string
}

/**
 * POST to a proxied agent route.
 * @param route message, stop or reset
 * @param body JSON body (only message takes one)
 * @returns ok flag, HTTP status and the server's message (or the network error text)
 */
export async function postAgent(route: 'message' | 'stop' | 'reset', body?: unknown): Promise<AgentActionResult> {
  try {
    const resp = await fetch(`/api/agent/${route}`, {
      method: 'POST',
      headers: body === undefined ? undefined : { 'Content-Type': 'application/json' },
      body: body === undefined ? undefined : JSON.stringify(body),
    })
    const data = (await resp.json().catch(() => ({}))) as { ok?: boolean; message?: string }
    return { ok: resp.ok && data.ok !== false, status: resp.status, message: data.message ?? (resp.ok ? '' : `HTTP ${resp.status}`) }
  } catch (e) {
    return { ok: false, status: 0, message: e instanceof Error ? e.message : 'network error' }
  }
}

/**
 * GET /api/agent/state.
 * @returns the agent state, or null when the agent is unreachable
 */
export async function fetchAgentState(): Promise<AgentInfo | null> {
  try {
    const resp = await fetch('/api/agent/state')
    return resp.ok ? ((await resp.json()) as AgentInfo) : null
  } catch {
    return null
  }
}
