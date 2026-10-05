/** Calls of the backend's claude_agent proxy (/api/agent/*). */
import { historyQuery } from './agentModel'
import type { AgentEvent, AgentInfo } from './agentModel'

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

export interface HistoryPage {
  events: AgentEvent[]
  has_more: boolean
}

/**
 * GET /api/agent/history: one page of events older than a cursor.
 * @param beforeSeq only events with a lower seq (the oldest loaded seq)
 * @param limit page size
 * @returns the page (ascending seq), or null when the agent is unreachable or answers with an error
 */
export async function fetchAgentHistory(beforeSeq: number, limit: number): Promise<HistoryPage | null> {
  try {
    const resp = await fetch(historyQuery(beforeSeq, limit))
    if (!resp.ok) return null
    const data = (await resp.json()) as Partial<HistoryPage>
    return Array.isArray(data.events) ? { events: data.events, has_more: data.has_more === true } : null
  } catch {
    return null
  }
}
