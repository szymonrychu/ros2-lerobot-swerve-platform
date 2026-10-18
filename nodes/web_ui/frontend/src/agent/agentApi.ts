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

export interface RetroPhase {
  name: string
  outcome: string
}

/** One job retrospective, as served by claude_agent GET /api/retrospectives. */
export interface Retrospective {
  id: string
  created_at: number
  instruction: string
  workflow: string | null
  outcome: string
  trivial: boolean
  author: 'agent' | 'runner'
  summary: string
  went_well: string[]
  went_wrong: string[]
  improvements: string[]
  notes_for_fixers: string[]
  skipped_mandatory: string[]
  phases: RetroPhase[]
  cost_usd: number | null
  num_turns: number | null
  effector_calls: number | null
  duration_s: number | null
}

export interface RetrospectivesPage {
  retrospectives: Retrospective[]
  total: number
}

/**
 * GET /api/agent/retrospectives: stored retrospectives, newest first.
 * @param limit page size (the server default when omitted)
 * @param before only records created before this unix timestamp (seconds)
 * @returns the page
 * @throws Error with the server's message (or HTTP status) when the agent is unreachable or answers with an error
 */
export async function fetchRetrospectives(limit?: number, before?: number): Promise<RetrospectivesPage> {
  const params = new URLSearchParams()
  if (limit !== undefined) params.set('limit', String(limit))
  if (before !== undefined) params.set('before', String(before))
  const query = params.toString()
  const resp = await fetch(`/api/agent/retrospectives${query ? `?${query}` : ''}`)
  const data = (await resp.json().catch(() => ({}))) as Partial<RetrospectivesPage> & { message?: string }
  if (!resp.ok) throw new Error(data.message ?? `HTTP ${resp.status}`)
  return { retrospectives: Array.isArray(data.retrospectives) ? data.retrospectives : [], total: data.total ?? 0 }
}
