/**
 * Pure logic of the Agent chat tab: reduces claude_agent events into a render model.
 *
 * Events are `{seq, ts, type, ...}` (see nodes/claude_agent/README.md). tool_call and tool_result are paired by id,
 * events are deduplicated by seq (the WebSocket replays the history on every connect), and the busy flag and
 * usage counters and the agent's phase plan follow `state` events; plan / phase events keep the plan current between them.
 */

export type ToolKind = 'sensor' | 'effector' | 'uncapped' | 'notes' | 'plan'

export type ToolContent =
  | { type: 'text'; text: string }
  | { type: 'image'; media_type: string; data_b64: string }

export interface AgentEvent {
  seq: number
  ts: number
  type: string
  [field: string]: unknown
}

export type PhaseStatus = 'pending' | 'active' | 'done' | 'failed' | 'skipped'

/** One phase of the agent's plan with its caps and usage (camelCase of the wire phase object). */
export interface PhaseInfo {
  index: number
  name: string
  goal: string
  status: PhaseStatus
  rwCap: number
  turnCap: number
  roUsed: number
  rwUsed: number
  turnsUsed: number
  raised: boolean
  summary: string
}

/** The plan the agent made for the running instruction (camelCase of the `plan` event / state payload). */
export interface PlanInfo {
  complexity: string
  rationale: string
  revised: boolean
  revisionRationale: string
  /** Index of the active phase; null before the first phase starts and after the last one was completed. */
  activePhase: number | null
  phases: PhaseInfo[]
}

/** GET /api/agent/state (claude_agent GET /api/state). */
export interface AgentInfo {
  busy: boolean
  model: string
  max_turns: number
  hard_max?: { rw_cap: number; turn_cap: number }
  phase_max?: { rw_cap: number; turn_cap: number }
  ro_used: number
  rw_used: number
  turns_used: number
  effector_calls_used?: number
  plan: unknown
  active_phase?: number | null
  session_started_at: number | null
}

export type RobotEventSeverity = 'info' | 'warning' | 'critical'

export interface ToolResult {
  isError: boolean
  content: ToolContent[]
  truncated: boolean
}

export type ChatItem =
  | { key: string; kind: 'user'; seq: number; text: string }
  | { key: string; kind: 'assistant'; seq: number; text: string }
  | {
      key: string
      kind: 'tool'
      seq: number
      id: string
      name: string
      fullName: string
      toolKind: ToolKind
      input: unknown
      result: ToolResult | null
      /** True for a result whose tool_call is not loaded (older than the window): rendered as a standalone result card. */
      orphan?: boolean
    }
  | { key: string; kind: 'denied'; seq: number; id: string | null; name: string; reason: string }
  | { key: string; kind: 'turn_end'; seq: number; status: string; costUsd: number; numTurns: number; effectorCalls: number }
  | { key: string; kind: 'error'; seq: number; message: string }
  | ({ key: string; kind: 'plan'; seq: number } & PlanInfo)
  | ({ key: string; kind: 'plan_revised'; seq: number } & PlanInfo)
  | ({ key: string; kind: 'phase_started'; seq: number } & Omit<PhaseInfo, 'status'>)
  | {
      key: string
      kind: 'phase_completed'
      seq: number
      index: number
      name: string
      goal: string
      outcome: string
      summary: string
      usage: { roUsed: number; rwUsed: number; turnsUsed: number }
      caps: { rwCap: number; turnCap: number }
    }
  | {
      key: string
      kind: 'robot_event'
      seq: number
      eventType: string
      severity: RobotEventSeverity
      source: string
      message: string
    }

export interface ChatState {
  items: ChatItem[]
  lastSeq: number
  busy: boolean
  /** Sensor (read-only) robot calls used in the current instruction. */
  roUsed: number
  /** Effector (read-write) robot calls used in the current instruction. */
  rwUsed: number
  turnsUsed: number
  /** The plan the agent made for the current instruction; null until it did. */
  plan: PlanInfo | null
  /** Lowest event seq held in memory: the `before_seq` cursor of the next older page; null while nothing is loaded. */
  firstSeq: number | null
  /** Whether older events exist on the agent beyond firstSeq. */
  hasMore: boolean
  /** Virtuoso's firstItemIndex: lowered when older items are prepended, raised when the oldest are dropped. */
  firstItemIndex: number
}

export interface HistoryFrame {
  type: 'history'
  events: AgentEvent[]
  has_more?: boolean
}

export type AgentFrame = HistoryFrame | AgentEvent

export type ParsedFrame = { kind: 'frame'; frame: AgentFrame } | { kind: 'proxy_error'; message: string }

export type StatusColor = 'success' | 'warning' | 'error' | 'default'

export type UsageLevel = 'ok' | 'warn' | 'full'

export interface UsageBar {
  key: 'rw' | 'turns'
  label: string
  used: number
  cap: number
  /** used / cap clamped to 0..1 (0 for a zero cap). */
  fraction: number
  level: UsageLevel
}

export interface PhaseRow {
  /** 1-based number shown to the user. */
  number: number
  name: string
  goal: string
  status: PhaseStatus
  active: boolean
  summary: string
  raised: boolean
  /** Sensor calls so far in the phase (a plain count, sensors have no cap). */
  sensorCalls: number
  bars: UsageBar[]
}

export interface UsageChip {
  key: 'ro' | 'rw' | 'turns'
  label: string
  level: UsageLevel
}

export interface HeaderStatus {
  model: string | null
  busy: boolean
  complexity: string | null
  /** The active phase (1-based number, phase count, name); null without a plan or after the last phase. */
  phase: { number: number; total: number; name: string } | null
  chips: UsageChip[]
}

export interface ComposerState {
  busy: boolean
  connected: boolean
  batteryCutoff: boolean
  batteryMessage: string | null
}

const TURN_STATUS: Record<string, { label: string; color: StatusColor }> = {
  done: { label: 'Done', color: 'success' },
  interrupted: { label: 'Interrupted', color: 'warning' },
  max_turns: { label: 'Turn cap reached', color: 'warning' },
  turn_cap: { label: 'Turn budget used up', color: 'warning' },
  error: { label: 'Error', color: 'error' },
}

/** Fraction of a cap from which a usage chip turns amber. */
const WARN_FRACTION = 0.8
const SEVERITIES: RobotEventSeverity[] = ['info', 'warning', 'critical']

/** Most items kept in memory; older ones are dropped and stay loadable from the agent. */
export const MAX_ITEMS = 400
/** Events per older-history page. */
export const HISTORY_PAGE_SIZE = 100
/** Starting firstItemIndex (positive and large, so prepending can lower it). */
export const FIRST_ITEM_INDEX = 1_000_000

export function emptyChat(): ChatState {
  return {
    items: [], lastSeq: 0, busy: false, roUsed: 0, rwUsed: 0, turnsUsed: 0, plan: null, firstSeq: null, hasMore: false,
    firstItemIndex: FIRST_ITEM_INDEX,
  }
}

function str(value: unknown, fallback = ''): string {
  return typeof value === 'string' ? value : fallback
}

function num(value: unknown, fallback = 0): number {
  return typeof value === 'number' ? value : fallback
}

const PHASE_STATUSES: PhaseStatus[] = ['pending', 'active', 'done', 'failed', 'skipped']

function parsePhase(raw: unknown, fallbackIndex: number): PhaseInfo {
  const p = (typeof raw === 'object' && raw !== null ? raw : {}) as Record<string, unknown>
  return {
    index: num(p.index, fallbackIndex),
    name: str(p.name),
    goal: str(p.goal),
    status: PHASE_STATUSES.find((s) => s === p.status) ?? 'pending',
    rwCap: num(p.rw_cap),
    turnCap: num(p.turn_cap),
    roUsed: num(p.ro_used),
    rwUsed: num(p.rw_used),
    turnsUsed: num(p.turns_used),
    raised: p.raised === true,
    summary: str(p.summary),
  }
}

/**
 * Read a plan object from an event or API payload.
 * @param raw the `plan` field (or the fields of a `plan` / `plan_revised` event)
 * @returns the plan, or null when raw is not an object with a complexity and a phases array
 */
export function parsePlan(raw: unknown): PlanInfo | null {
  if (typeof raw !== 'object' || raw === null) return null
  const p = raw as Record<string, unknown>
  if (typeof p.complexity !== 'string' || !Array.isArray(p.phases)) return null
  return {
    complexity: p.complexity,
    rationale: str(p.rationale),
    revised: p.revised === true,
    revisionRationale: str(p.revision_rationale),
    activePhase: typeof p.active_phase === 'number' ? p.active_phase : null,
    phases: p.phases.map((phase, i) => parsePhase(phase, i)),
  }
}

function withPhase(plan: PlanInfo | null, index: number, update: (phase: PhaseInfo) => PhaseInfo, activePhase: number | null): PlanInfo | null {
  if (plan === null) return null
  return { ...plan, activePhase, phases: plan.phases.map((phase) => (phase.index === index ? update(phase) : phase)) }
}

/**
 * Apply one event to the chat state.
 * @param state current state (not mutated)
 * @param event event from the agent; one with a seq at or below state.lastSeq is a duplicate and ignored
 * @returns the next state
 */
export function reduceEvent(state: ChatState, event: AgentEvent): ChatState {
  if (event.seq <= state.lastSeq) return state
  const base = { key: `e${event.seq}`, seq: event.seq }
  const next: ChatState = { ...state, lastSeq: event.seq, firstSeq: state.firstSeq ?? event.seq }
  switch (event.type) {
    case 'user_message':
      return { ...next, items: [...state.items, { ...base, kind: 'user', text: str(event.text) }] }
    case 'assistant_text':
      return { ...next, items: [...state.items, { ...base, kind: 'assistant', text: str(event.text) }] }
    case 'tool_call':
      return {
        ...next,
        items: [
          ...state.items,
          {
            ...base,
            kind: 'tool',
            id: str(event.id),
            name: str(event.name),
            fullName: str(event.full_name, str(event.name)),
            toolKind: (str(event.kind, 'uncapped') as ToolKind),
            input: event.input ?? {},
            result: null,
          },
        ],
      }
    case 'tool_result': {
      const id = str(event.id)
      const result: ToolResult = {
        isError: event.is_error === true,
        content: Array.isArray(event.content) ? (event.content as ToolContent[]) : [],
        truncated: event.truncated === true,
      }
      const paired = state.items.some((item) => item.kind === 'tool' && item.id === id && item.result === null)
      if (!paired) {
        const orphan: ChatItem = {
          ...base, kind: 'tool', id, name: 'tool result', fullName: 'tool result', toolKind: 'uncapped', input: {}, result, orphan: true,
        }
        return { ...next, items: [...state.items, orphan] }
      }
      const items = state.items.map((item) => (item.kind === 'tool' && item.id === id && item.result === null ? { ...item, result } : item))
      return { ...next, items }
    }
    case 'tool_denied':
      return {
        ...next,
        items: [
          ...state.items,
          { ...base, kind: 'denied', id: typeof event.id === 'string' ? event.id : null, name: str(event.name), reason: str(event.reason) },
        ],
      }
    case 'turn_end':
      return {
        ...next,
        items: [
          ...state.items,
          {
            ...base,
            kind: 'turn_end',
            status: str(event.status),
            costUsd: num(event.cost_usd),
            numTurns: num(event.num_turns),
            effectorCalls: num(event.effector_calls),
          },
        ],
      }
    case 'error':
      return { ...next, items: [...state.items, { ...base, kind: 'error', message: str(event.message) }] }
    case 'state':
      return {
        ...next,
        busy: event.busy === true,
        roUsed: num(event.ro_used, state.roUsed),
        rwUsed: num(event.rw_used, state.rwUsed),
        turnsUsed: num(event.turns_used, state.turnsUsed),
        plan: 'plan' in event ? parsePlan(event.plan) : state.plan,
      }
    case 'plan':
    case 'plan_revised': {
      const plan = parsePlan(event)
      if (!plan) return next
      const kind = event.type === 'plan' ? 'plan' : 'plan_revised'
      return { ...next, plan, items: [...state.items, { ...base, kind, ...plan }] }
    }
    case 'phase_started': {
      const phase = parsePhase(event, num(event.index))
      const { status: _status, ...rest } = phase
      return {
        ...next,
        plan: withPhase(state.plan, phase.index, (p) => ({ ...p, status: 'active' }), phase.index),
        items: [...state.items, { ...base, kind: 'phase_started', ...rest }],
      }
    }
    case 'phase_completed': {
      const usage = (typeof event.usage === 'object' && event.usage !== null ? event.usage : {}) as Record<string, unknown>
      const caps = (typeof event.caps === 'object' && event.caps !== null ? event.caps : {}) as Record<string, unknown>
      const index = num(event.index)
      const outcome = str(event.outcome)
      const summary = str(event.summary)
      const status = PHASE_STATUSES.find((s) => s === outcome) ?? 'done'
      const item: ChatItem = {
        ...base,
        kind: 'phase_completed',
        index,
        name: str(event.name),
        goal: str(event.goal),
        outcome,
        summary,
        usage: { roUsed: num(usage.ro_used), rwUsed: num(usage.rw_used), turnsUsed: num(usage.turns_used) },
        caps: { rwCap: num(caps.rw_cap), turnCap: num(caps.turn_cap) },
      }
      const plan = withPhase(
        state.plan,
        index,
        (p) => ({ ...p, status, summary, roUsed: num(usage.ro_used, p.roUsed), rwUsed: num(usage.rw_used, p.rwUsed), turnsUsed: num(usage.turns_used, p.turnsUsed) }),
        state.plan?.activePhase === index ? null : (state.plan?.activePhase ?? null),
      )
      return { ...next, plan, items: [...state.items, item] }
    }
    case 'robot_event': {
      const severity = SEVERITIES.find((s) => s === event.severity) ?? 'info'
      const item: ChatItem = {
        ...base, kind: 'robot_event', eventType: str(event.event_type), severity, source: str(event.source), message: str(event.message),
      }
      return { ...next, items: [...state.items, item] }
    }
    default:
      return next
  }
}

function isHistory(frame: AgentFrame): frame is HistoryFrame {
  return frame.type === 'history' && Array.isArray((frame as HistoryFrame).events)
}

/**
 * Apply a WebSocket frame: a history frame replaces the transcript (the newest page, replayed on connect), any other
 * frame is one live event.
 * @param state current state (not mutated)
 * @param frame parsed frame
 * @param maxItems window for live events: the oldest items beyond it are dropped; null keeps everything (the user is reading older items)
 * @returns the next state
 */
export function applyFrame(state: ChatState, frame: AgentFrame, maxItems: number | null = MAX_ITEMS): ChatState {
  if (!isHistory(frame)) {
    const next = reduceEvent(state, frame)
    return maxItems === null ? next : trimWindow(next, maxItems)
  }
  const replayed = frame.events.reduce(reduceEvent, emptyChat())
  const paged = { ...replayed, firstSeq: frame.events[0]?.seq ?? null, hasMore: frame.has_more === true }
  const hasState = frame.events.some((e) => e.type === 'state')
  return hasState
    ? paged
    : { ...paged, busy: state.busy, roUsed: state.roUsed, rwUsed: state.rwUsed, turnsUsed: state.turnsUsed, plan: state.plan }
}

/**
 * Drop the oldest items beyond the window; they stay loadable from the agent.
 * @param state current state
 * @param maxItems most items to keep
 * @returns the same state when it fits, else one with the newest maxItems items, the cursor moved to the first kept
 *   item, hasMore set and firstItemIndex raised by the number dropped
 */
export function trimWindow(state: ChatState, maxItems: number = MAX_ITEMS): ChatState {
  const drop = state.items.length - maxItems
  if (drop <= 0) return state
  const items = state.items.slice(drop)
  return { ...state, items, firstSeq: items[0].seq, hasMore: true, firstItemIndex: state.firstItemIndex + drop }
}

/**
 * Merge an older history page in front of the loaded items.
 * Events at or above the cursor are duplicates and skipped. A tool_call on the page whose result is already loaded as a
 * standalone result is paired with it; a result on the page without its call stays a standalone result.
 * @param state current state (not mutated)
 * @param events older events, ascending seq
 * @param hasMore the agent's has_more for this page
 * @returns the next state
 */
export function prependHistory(state: ChatState, events: AgentEvent[], hasMore: boolean): ChatState {
  const cursor = state.firstSeq ?? Number.POSITIVE_INFINITY
  const fresh = events.filter((e) => e.seq < cursor)
  const older = fresh.reduce(reduceEvent, emptyChat()).items
  const orphans = new Map<string, ChatItem>()
  for (const item of state.items) if (item.kind === 'tool' && item.orphan) orphans.set(item.id, item)
  const claimed = new Set<ChatItem>()
  const merged = older.map((item) => {
    if (item.kind !== 'tool' || item.result !== null) return item
    const orphan = orphans.get(item.id)
    if (!orphan || orphan.kind !== 'tool' || claimed.has(orphan)) return item
    claimed.add(orphan)
    return { ...item, result: orphan.result }
  })
  const items = [...merged, ...state.items.filter((item) => !claimed.has(item))]
  return {
    ...state,
    items,
    firstSeq: fresh.length > 0 ? fresh[0].seq : state.firstSeq,
    hasMore,
    firstItemIndex: state.firstItemIndex - (items.length - state.items.length),
  }
}

/**
 * URL of one older-history page through the backend proxy.
 * @param beforeSeq cursor (only events with a lower seq), null for the newest page
 * @param limit page size
 * @returns the request URL
 */
export function historyQuery(beforeSeq: number | null, limit: number): string {
  const params = new URLSearchParams()
  if (beforeSeq !== null) params.set('before_seq', String(beforeSeq))
  params.set('limit', String(limit))
  return `/api/agent/history?${params.toString()}`
}

/**
 * Parse a /ws/agent text frame.
 * @param raw frame text
 * @returns a history frame or event, the proxy's seq-less error frame, or null when it is not valid
 */
export function parseAgentFrame(raw: string): ParsedFrame | null {
  let data: unknown
  try {
    data = JSON.parse(raw)
  } catch {
    return null
  }
  if (typeof data !== 'object' || data === null || Array.isArray(data)) return null
  const obj = data as Record<string, unknown>
  if (typeof obj.type !== 'string') return null
  if (obj.type === 'history') return Array.isArray(obj.events) ? { kind: 'frame', frame: obj as unknown as HistoryFrame } : null
  if (typeof obj.seq !== 'number') {
    return obj.type === 'error' ? { kind: 'proxy_error', message: str(obj.message, 'agent error') } : null
  }
  return { kind: 'frame', frame: obj as AgentEvent }
}

/**
 * Take busy, the usage counters and the plan from a /api/agent/state snapshot.
 * @param state current state
 * @param info state snapshot
 * @returns the next state
 */
export function applyStateSnapshot(state: ChatState, info: AgentInfo): ChatState {
  return {
    ...state,
    busy: info.busy,
    roUsed: info.ro_used,
    rwUsed: info.rw_used,
    turnsUsed: info.turns_used,
    plan: parsePlan(info.plan),
  }
}

/**
 * Label and colour of a turn_end status.
 * @param status status string from the event
 * @returns display label and MUI colour
 */
export function statusLabel(status: string): { label: string; color: StatusColor } {
  return TURN_STATUS[status] ?? { label: status, color: 'default' }
}

/**
 * Colour level of a usage chip.
 * @param used calls or turns used
 * @param cap the agent-set cap, null while no budget is set
 * @returns "full" at or above the cap, "warn" from 80 percent of it, else "ok" (also without a cap or with a zero cap)
 */
export function usageLevel(used: number, cap: number | null): UsageLevel {
  if (cap === null || cap <= 0) return 'ok'
  if (used >= cap) return 'full'
  return used >= cap * WARN_FRACTION ? 'warn' : 'ok'
}

function usageChip(key: UsageChip['key'], name: string, used: number, cap: number | null): UsageChip {
  return { key, label: cap === null ? `${name} ${used}` : `${name} ${used} / ${cap}`, level: usageLevel(used, cap) }
}

function usageBar(key: UsageBar['key'], name: string, used: number, cap: number): UsageBar {
  return { key, label: `${name} ${used} / ${cap}`, used, cap, fraction: cap > 0 ? Math.min(used / cap, 1) : 0, level: usageLevel(used, cap) }
}

/**
 * Rows of the phase list (side panel / header).
 * @param plan the agent's plan, null before it set one
 * @returns one row per phase with its status, whether it is the active one, its sensor call count and rw / turns usage bars (amber from
 *   80 percent of a cap, red at the cap); empty without a plan
 */
export function phaseRows(plan: PlanInfo | null): PhaseRow[] {
  if (plan === null) return []
  return plan.phases.map((p) => ({
    number: p.index + 1,
    name: p.name,
    goal: p.goal,
    status: p.status,
    active: p.status === 'active' && plan.activePhase === p.index,
    summary: p.summary,
    raised: p.raised,
    sensorCalls: p.roUsed,
    bars: [usageBar('rw', 'rw', p.rwUsed, p.rwCap), usageBar('turns', 'turns', p.turnsUsed, p.turnCap)],
  }))
}

/**
 * Values of the header strip.
 * @param info last /api/agent/state snapshot, null until loaded
 * @param state live chat state (busy, counters and plan follow state events)
 * @returns model, busy flag, the complexity the agent judged (null before it made a plan), the active phase and the
 *   sensor count (no cap) and rw / turns chips of the active phase ("used / cap"); without an active phase the instruction totals, without caps
 */
export function headerStatus(info: AgentInfo | null, state: ChatState): HeaderStatus {
  const plan = state.plan
  const active = plan && plan.activePhase !== null ? (plan.phases.find((p) => p.index === plan.activePhase) ?? null) : null
  return {
    model: info?.model ?? null,
    busy: state.busy,
    complexity: plan?.complexity ?? null,
    phase: plan && active ? { number: active.index + 1, total: plan.phases.length, name: active.name } : null,
    chips: active
      ? [
          usageChip('ro', 'sensors', active.roUsed, null),
          usageChip('rw', 'rw', active.rwUsed, active.rwCap),
          usageChip('turns', 'turns', active.turnsUsed, active.turnCap),
        ]
      : [
          usageChip('ro', 'sensors', state.roUsed, null),
          usageChip('rw', 'rw', state.rwUsed, null),
          usageChip('turns', 'turns', state.turnsUsed, null),
        ],
  }
}

/**
 * Why the composer is disabled.
 * @param s busy flag, agent connection and battery cut-off state
 * @returns the reason text (battery message first), or null when sending is allowed
 */
export function composerBlockReason(s: ComposerState): string | null {
  if (s.batteryCutoff) return s.batteryMessage ?? 'Battery below cut-off: the agent cannot be commanded'
  if (!s.connected) return 'Agent not connected'
  if (s.busy) return 'Agent is working'
  return null
}
