/**
 * Pure logic of the Agent chat tab: reduces claude_agent events into a render model.
 *
 * Events are `{seq, ts, type, ...}` (see nodes/claude_agent/README.md). tool_call and tool_result are paired by id,
 * events are deduplicated by seq (the WebSocket replays the history on every connect), and the busy flag and
 * effector counter follow `state` events.
 */

export type ToolKind = 'sensor' | 'effector' | 'uncapped'

export type ToolContent =
  | { type: 'text'; text: string }
  | { type: 'image'; media_type: string; data_b64: string }

export interface AgentEvent {
  seq: number
  ts: number
  type: string
  [field: string]: unknown
}

/** GET /api/agent/state (claude_agent GET /api/state). */
export interface AgentInfo {
  busy: boolean
  model: string
  max_turns: number
  effector_call_cap: number
  effector_calls_used: number
  session_started_at: number | null
}

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
    }
  | { key: string; kind: 'denied'; seq: number; id: string | null; name: string; reason: string }
  | { key: string; kind: 'turn_end'; seq: number; status: string; costUsd: number; numTurns: number; effectorCalls: number }
  | { key: string; kind: 'error'; seq: number; message: string }

export interface ChatState {
  items: ChatItem[]
  lastSeq: number
  busy: boolean
  effectorCallsUsed: number
}

export interface HistoryFrame {
  type: 'history'
  events: AgentEvent[]
}

export type AgentFrame = HistoryFrame | AgentEvent

export type ParsedFrame = { kind: 'frame'; frame: AgentFrame } | { kind: 'proxy_error'; message: string }

export type StatusColor = 'success' | 'warning' | 'error' | 'default'

export interface HeaderStatus {
  model: string | null
  busy: boolean
  effectorLabel: string
  maxTurns: number | null
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
  error: { label: 'Error', color: 'error' },
}

export function emptyChat(): ChatState {
  return { items: [], lastSeq: 0, busy: false, effectorCallsUsed: 0 }
}

function str(value: unknown, fallback = ''): string {
  return typeof value === 'string' ? value : fallback
}

function num(value: unknown, fallback = 0): number {
  return typeof value === 'number' ? value : fallback
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
  const next: ChatState = { ...state, lastSeq: event.seq }
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
      return { ...next, busy: event.busy === true, effectorCallsUsed: num(event.effector_calls_used, state.effectorCallsUsed) }
    default:
      return next
  }
}

function isHistory(frame: AgentFrame): frame is HistoryFrame {
  return frame.type === 'history' && Array.isArray((frame as HistoryFrame).events)
}

/**
 * Apply a WebSocket frame: a history frame replaces the transcript (events replayed on connect), any other frame is one event.
 * @param state current state (not mutated)
 * @param frame parsed frame
 * @returns the next state
 */
export function applyFrame(state: ChatState, frame: AgentFrame): ChatState {
  if (!isHistory(frame)) return reduceEvent(state, frame)
  const replayed = frame.events.reduce(reduceEvent, { ...emptyChat() })
  const hasState = frame.events.some((e) => e.type === 'state')
  return hasState ? replayed : { ...replayed, busy: state.busy, effectorCallsUsed: state.effectorCallsUsed }
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
 * Take busy and the effector counter from a /api/agent/state snapshot.
 * @param state current state
 * @param info state snapshot
 * @returns the next state
 */
export function applyStateSnapshot(state: ChatState, info: AgentInfo): ChatState {
  return { ...state, busy: info.busy, effectorCallsUsed: info.effector_calls_used }
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
 * Values of the header strip.
 * @param info last /api/agent/state snapshot, null until loaded
 * @param state live chat state (busy and counter follow state events)
 * @returns model, busy flag, "used / cap" label (just "used" without the cap) and turn cap
 */
export function headerStatus(info: AgentInfo | null, state: ChatState): HeaderStatus {
  return {
    model: info?.model ?? null,
    busy: state.busy,
    effectorLabel: info ? `${state.effectorCallsUsed} / ${info.effector_call_cap}` : `${state.effectorCallsUsed}`,
    maxTurns: info?.max_turns ?? null,
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
