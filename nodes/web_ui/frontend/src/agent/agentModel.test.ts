import { describe, expect, it } from 'vitest'
import {
  applyFrame,
  applyStateSnapshot,
  composerBlockReason,
  emptyChat,
  FIRST_ITEM_INDEX,
  MAX_ITEMS,
  headerStatus,
  historyQuery,
  prependHistory,
  trimWindow,
  usageLevel,
  parseAgentFrame,
  parsePlan,
  phaseRows,
  reduceEvent,
  statusLabel,
} from './agentModel'
import type { AgentEvent, ChatState } from './agentModel'

const ev = (seq: number, type: string, extra: Record<string, unknown> = {}): AgentEvent =>
  ({ seq, ts: seq, type, ...extra }) as AgentEvent

const TOOL_CALL = ev(2, 'tool_call', { id: 't1', name: 'get_battery', full_name: 'mcp__robot__get_battery', kind: 'sensor', input: { a: 1 } })

function run(events: AgentEvent[], start: ChatState = emptyChat()): ChatState {
  return events.reduce(reduceEvent, start)
}

const phase = (index: number, name: string, goal: string, status: string, caps: number[], used: number[]) => ({
  index, name, goal, status, rw_cap: caps[0], turn_cap: caps[1], ro_used: used[0], rw_used: used[1], turns_used: used[2],
  raised: false, summary: status === 'done' ? 'found' : '',
})
const PLAN_PAYLOAD = {
  complexity: 'complex', rationale: 'pick and place', revised: false, revision_rationale: '', active_phase: 1,
  phases: [
    phase(0, 'Locate', 'tomato seen', 'done', [3, 10], [9, 1, 4]),
    phase(1, 'Drive', 'within 10 cm', 'active', [10, 12], [6, 4, 3]),
    phase(2, 'Pick', 'tomato held', 'pending', [20, 25], [0, 0, 0]),
  ],
}

describe('reduceEvent', () => {
  it('builds user and assistant items in order', () => {
    const s = run([ev(1, 'user_message', { text: 'hi' }), ev(2, 'assistant_text', { text: 'hello\nthere' })])
    expect(s.items.map((i) => i.kind)).toEqual(['user', 'assistant'])
    expect(s.items[1]).toMatchObject({ kind: 'assistant', text: 'hello\nthere' })
    expect(s.lastSeq).toBe(2)
  })

  it('pairs tool_result with its tool_call by id', () => {
    const s = run([
      TOOL_CALL,
      ev(3, 'tool_result', { id: 't1', is_error: false, content: [{ type: 'text', text: 'ok 12V' }], truncated: true }),
    ])
    expect(s.items).toHaveLength(1)
    const tool = s.items[0]
    expect(tool).toMatchObject({ kind: 'tool', id: 't1', name: 'get_battery', toolKind: 'sensor', input: { a: 1 } })
    if (tool.kind !== 'tool') throw new Error('not a tool')
    expect(tool.result).toEqual({ isError: false, content: [{ type: 'text', text: 'ok 12V' }], truncated: true })
  })

  it('keeps a tool call pending until its result arrives', () => {
    const s = run([TOOL_CALL])
    const tool = s.items[0]
    if (tool.kind !== 'tool') throw new Error('not a tool')
    expect(tool.result).toBeNull()
  })

  it('renders a tool_result without a loaded call as a standalone result item', () => {
    const s = run([ev(1, 'tool_result', { id: 'zzz', is_error: false, content: [{ type: 'text', text: 'x' }], truncated: false })])
    expect(s.items).toHaveLength(1)
    expect(s.items[0]).toMatchObject({ kind: 'tool', id: 'zzz', orphan: true, result: { isError: false } })
    expect(s.lastSeq).toBe(1)
  })

  it('maps the notes tool kind', () => {
    const s = run([ev(1, 'tool_call', { id: 'n', name: 'write_note', kind: 'notes', input: {} })])
    expect(s.items[0]).toMatchObject({ toolKind: 'notes' })
  })

  it('dedupes by seq', () => {
    const e = ev(1, 'assistant_text', { text: 'x' })
    const s = run([e, e, ev(1, 'assistant_text', { text: 'other' })])
    expect(s.items).toHaveLength(1)
  })

  it('adds denied, turn_end and error items', () => {
    const s = run([
      ev(1, 'tool_denied', { id: null, name: 'mcp__robot__drive', reason: 'cap reached' }),
      ev(2, 'turn_end', { status: 'done', cost_usd: 0.1234, num_turns: 3, effector_calls: 2 }),
      ev(3, 'error', { message: 'bad token' }),
    ])
    expect(s.items.map((i) => i.kind)).toEqual(['denied', 'turn_end', 'error'])
    expect(s.items[1]).toMatchObject({ status: 'done', costUsd: 0.1234, numTurns: 3, effectorCalls: 2 })
  })

  it('state events update busy, usage and plan without adding items', () => {
    const s = run([ev(1, 'state', { busy: true, ro_used: 3, rw_used: 4, turns_used: 2, plan: PLAN_PAYLOAD, active_phase: 1 })])
    expect(s.items).toHaveLength(0)
    expect(s.busy).toBe(true)
    expect([s.roUsed, s.rwUsed, s.turnsUsed]).toEqual([3, 4, 2])
    expect(s.plan?.activePhase).toBe(1)
    expect(s.plan?.phases.map((p) => p.status)).toEqual(['done', 'active', 'pending'])
    expect(s.plan?.phases[1]).toMatchObject({ name: 'Drive', goal: 'within 10 cm', rwCap: 10, turnCap: 12, roUsed: 6, rwUsed: 4, turnsUsed: 3 })
  })

  it('a state event with a null plan clears it (new instruction)', () => {
    const s = run([ev(1, 'state', { busy: true, plan: PLAN_PAYLOAD }), ev(2, 'state', { busy: true, ro_used: 0, plan: null })])
    expect(s.plan).toBeNull()
  })

  it('a plan event adds a card and sets the plan', () => {
    const s = run([ev(1, 'plan', PLAN_PAYLOAD)])
    expect(s.items).toHaveLength(1)
    expect(s.items[0]).toMatchObject({ kind: 'plan', complexity: 'complex', rationale: 'pick and place' })
    expect(s.items[0].kind === 'plan' && s.items[0].phases.map((p) => p.name)).toEqual(['Locate', 'Drive', 'Pick'])
    expect(s.plan).toMatchObject({ complexity: 'complex', activePhase: 1 })
  })

  it('phase_started adds a compact card and marks the phase active in the plan', () => {
    const base = run([ev(1, 'plan', { ...PLAN_PAYLOAD, active_phase: 0, phases: PLAN_PAYLOAD.phases.map((p, i) => ({ ...p, status: i === 0 ? 'active' : 'pending' })) })])
    const s = run([ev(2, 'phase_started', { ...PLAN_PAYLOAD.phases[1], status: 'active' })], base)
    expect(s.items[1]).toMatchObject({ kind: 'phase_started', index: 1, name: 'Drive', goal: 'within 10 cm', rwCap: 10, turnCap: 12 })
    expect(s.items[1]).not.toHaveProperty('roCap')
    expect(s.plan?.activePhase).toBe(1)
    expect(s.plan?.phases.map((p) => p.status)).toEqual(['active', 'active', 'pending'])
  })

  it('phase_completed adds a card with outcome, summary, usage and caps and closes the phase in the plan', () => {
    const base = run([ev(1, 'plan', { ...PLAN_PAYLOAD, active_phase: 0, phases: PLAN_PAYLOAD.phases.map((p, i) => ({ ...p, status: i === 0 ? 'active' : 'pending' })) })])
    const s = run([
      ev(2, 'phase_completed', {
        index: 0, name: 'Locate', goal: 'tomato seen', outcome: 'failed', summary: 'no tomato',
        usage: { ro_used: 12, rw_used: 1, turns_used: 6 }, caps: { rw_cap: 3, turn_cap: 10 },
      }),
    ], base)
    expect(s.items[1]).toMatchObject({
      kind: 'phase_completed', index: 0, name: 'Locate', outcome: 'failed', summary: 'no tomato',
      usage: { roUsed: 12, rwUsed: 1, turnsUsed: 6 }, caps: { rwCap: 3, turnCap: 10 },
    })
    expect(s.plan?.phases[0]).toMatchObject({ status: 'failed', summary: 'no tomato', roUsed: 12 })
  })

  it('plan_revised adds a card and replaces the plan', () => {
    const s = run([ev(1, 'plan', PLAN_PAYLOAD), ev(2, 'plan_revised', { ...PLAN_PAYLOAD, revised: true, revision_rationale: 'try another way' })])
    expect(s.items[1]).toMatchObject({ kind: 'plan_revised', revisionRationale: 'try another way' })
    expect(s.plan).toMatchObject({ revised: true, revisionRationale: 'try another way' })
  })

  it('maps the plan tool kind', () => {
    const s = run([ev(1, 'tool_call', { id: 'b', name: 'set_task_plan', kind: 'plan', input: {} })])
    expect(s.items[0]).toMatchObject({ toolKind: 'plan' })
  })

  it('robot_event adds a coloured alert item', () => {
    const s = run([
      ev(1, 'robot_event', { event_seq: 3, event_ts: 1, event_type: 'bump', severity: 'critical', source: 'mcp_server', message: 'hit', data: {} }),
    ])
    expect(s.items[0]).toEqual({
      key: 'e1', seq: 1, kind: 'robot_event', eventType: 'bump', severity: 'critical', source: 'mcp_server', message: 'hit',
    })
  })

  it('robot_event with an unknown severity is shown as info', () => {
    const s = run([ev(1, 'robot_event', { event_type: 'x', severity: 'odd', message: 'm' })])
    expect(s.items[0]).toMatchObject({ severity: 'info' })
  })

  it('does not mutate the previous state', () => {
    const before = run([TOOL_CALL])
    const snapshot = JSON.stringify(before)
    reduceEvent(before, ev(3, 'tool_result', { id: 't1', is_error: true, content: [], truncated: false }))
    expect(JSON.stringify(before)).toBe(snapshot)
  })
})

describe('applyFrame', () => {
  it('history replaces the items and lastSeq, keeping live state fields', () => {
    const start = { ...run([ev(5, 'assistant_text', { text: 'old' })]), busy: true }
    const s = applyFrame(start, { type: 'history', events: [ev(1, 'user_message', { text: 'a' }), ev(2, 'assistant_text', { text: 'b' })] })
    expect(s.items.map((i) => i.kind)).toEqual(['user', 'assistant'])
    expect(s.lastSeq).toBe(2)
    expect(s.busy).toBe(true)
  })

  it('history replay then overlapping live events do not duplicate', () => {
    let s = applyFrame(emptyChat(), { type: 'history', events: [ev(1, 'user_message', { text: 'a' })] })
    s = applyFrame(s, ev(1, 'user_message', { text: 'a' }))
    s = applyFrame(s, ev(2, 'assistant_text', { text: 'b' }))
    expect(s.items).toHaveLength(2)
  })

  it('history derives busy from state events inside it', () => {
    const s = applyFrame(emptyChat(), { type: 'history', events: [ev(1, 'state', { busy: true, rw_used: 1 })] })
    expect(s.busy).toBe(true)
    expect(s.rwUsed).toBe(1)
  })
})

describe('parseAgentFrame', () => {
  it('parses history, events and the proxy disconnect error', () => {
    expect(parseAgentFrame('{"type":"history","events":[]}')).toEqual({ kind: 'frame', frame: { type: 'history', events: [] } })
    expect(parseAgentFrame('{"seq":1,"ts":1,"type":"state","busy":false}')).toMatchObject({ kind: 'frame' })
    expect(parseAgentFrame('{"type":"error","message":"agent disconnected"}')).toEqual({ kind: 'proxy_error', message: 'agent disconnected' })
  })

  it('returns null for garbage', () => {
    expect(parseAgentFrame('nope')).toBeNull()
    expect(parseAgentFrame('{"x":1}')).toBeNull()
    expect(parseAgentFrame('[]')).toBeNull()
  })
})

const INFO = {
  busy: false, model: 'opus', max_turns: 160, hard_max: { rw_cap: 150, turn_cap: 200 },
  ro_used: 0, rw_used: 0, turns_used: 0, effector_calls_used: 0, plan: null, active_phase: null, session_started_at: 1,
}

describe('parsePlan', () => {
  it('reads a plan payload and rejects garbage', () => {
    expect(parsePlan(PLAN_PAYLOAD)?.phases).toHaveLength(3)
    expect(parsePlan(null)).toBeNull()
    expect(parsePlan({ complexity: 'x' })).toBeNull()
    expect(parsePlan('plan')).toBeNull()
  })

  it('falls back to pending for an unknown status', () => {
    const p = parsePlan({ ...PLAN_PAYLOAD, phases: [phase(0, 'A', 'g', 'weird', [0, 1], [0, 0, 0])] })
    expect(p?.phases[0].status).toBe('pending')
  })
})

describe('applyStateSnapshot', () => {
  it('copies busy, usage and plan', () => {
    const s = applyStateSnapshot(emptyChat(), { ...INFO, busy: true, ro_used: 7, rw_used: 3, turns_used: 5, plan: PLAN_PAYLOAD, active_phase: 1 })
    expect(s.busy).toBe(true)
    expect([s.roUsed, s.rwUsed, s.turnsUsed]).toEqual([7, 3, 5])
    expect(s.plan).toMatchObject({ complexity: 'complex', activePhase: 1 })
  })
})

describe('history frames keep the live usage when they hold no state event', () => {
  it('keeps plan and counters', () => {
    const live = run([ev(1, 'state', { busy: true, ro_used: 2, rw_used: 1, turns_used: 3, plan: PLAN_PAYLOAD })])
    const s = applyFrame(live, { type: 'history', events: [ev(1, 'user_message', { text: 'a' })] })
    expect([s.roUsed, s.rwUsed, s.turnsUsed]).toEqual([2, 1, 3])
    expect(s.plan).not.toBeNull()
  })
})

describe('usageLevel', () => {
  it('is ok below 80 percent, warn from 80 percent and full at the cap', () => {
    expect(usageLevel(7, 10)).toBe('ok')
    expect(usageLevel(8, 10)).toBe('warn')
    expect(usageLevel(9, 10)).toBe('warn')
    expect(usageLevel(10, 10)).toBe('full')
    expect(usageLevel(11, 10)).toBe('full')
  })

  it('has no level without a cap and for a zero cap', () => {
    expect(usageLevel(5, null)).toBe('ok')
    expect(usageLevel(0, 0)).toBe('ok')
  })
})

describe('phaseRows', () => {
  it('lists every phase with status, active flag and usage bars (amber at 80 percent, red at the cap)', () => {
    const plan = parsePlan({
      ...PLAN_PAYLOAD,
      phases: [
        phase(0, 'Locate', 'tomato seen', 'done', [3, 10], [9, 1, 4]),
        phase(1, 'Drive', 'within 10 cm', 'active', [5, 20], [8, 5, 3]),
        phase(2, 'Pick', 'tomato held', 'pending', [0, 25], [0, 0, 0]),
      ],
    })
    const rows = phaseRows(plan)
    expect(rows.map((r) => [r.number, r.status, r.active])).toEqual([[1, 'done', false], [2, 'active', true], [3, 'pending', false]])
    expect(rows[1].bars).toEqual([
      { key: 'rw', label: 'rw 5 / 5', used: 5, cap: 5, fraction: 1, level: 'full' },
      { key: 'turns', label: 'turns 3 / 20', used: 3, cap: 20, fraction: 0.15, level: 'ok' },
    ])
    expect(rows[2].bars[0]).toMatchObject({ label: 'rw 0 / 0', fraction: 0, level: 'ok' })
    expect(rows.map((r) => r.bars.map((b) => b.key))).toEqual([['rw', 'turns'], ['rw', 'turns'], ['rw', 'turns']])
    expect(rows.map((r) => r.sensorCalls)).toEqual([9, 8, 0])
    expect(rows[0]).toMatchObject({ name: 'Locate', goal: 'tomato seen', summary: 'found' })
  })

  it('clamps the bar fraction at 1 when usage exceeds the cap and returns [] without a plan', () => {
    const plan = parsePlan({ ...PLAN_PAYLOAD, phases: [phase(0, 'A', 'g', 'active', [1, 2], [0, 5, 0])] })
    expect(phaseRows(plan)[0].bars[0]).toMatchObject({ fraction: 1, level: 'full' })
    expect(phaseRows(null)).toEqual([])
  })
})

describe('statusLabel / headerStatus', () => {
  it('maps turn statuses', () => {
    expect(statusLabel('done')).toEqual({ label: 'Done', color: 'success' })
    expect(statusLabel('interrupted')).toEqual({ label: 'Interrupted', color: 'warning' })
    expect(statusLabel('max_turns')).toEqual({ label: 'Turn cap reached', color: 'warning' })
    expect(statusLabel('turn_cap')).toEqual({ label: 'Turn budget used up', color: 'warning' })
    expect(statusLabel('error')).toEqual({ label: 'Error', color: 'error' })
    expect(statusLabel('weird')).toEqual({ label: 'weird', color: 'default' })
  })

  it('shows no caps and no phase until the agent set a plan', () => {
    const s = { ...emptyChat(), busy: true, roUsed: 2, rwUsed: 1, turnsUsed: 3 }
    expect(headerStatus(INFO, s)).toEqual({
      model: 'opus',
      busy: true,
      complexity: null,
      phase: null,
      chips: [
        { key: 'ro', label: 'sensors 2', level: 'ok' },
        { key: 'rw', label: 'rw 1', level: 'ok' },
        { key: 'turns', label: 'turns 3', level: 'ok' },
      ],
    })
    expect(headerStatus(null, s).model).toBeNull()
  })

  it('shows the active phase and its used / cap chips (sensor calls are a plain number) with amber at 80 percent and red at the cap', () => {
    const plan = parsePlan({
      ...PLAN_PAYLOAD,
      active_phase: 1,
      phases: [
        phase(0, 'Locate', 'tomato seen', 'done', [3, 10], [9, 1, 4]),
        phase(1, 'Drive', 'within 10 cm', 'active', [5, 20], [8, 5, 3]),
        phase(2, 'Pick', 'tomato held', 'pending', [20, 25], [0, 0, 0]),
      ],
    })
    const s = { ...emptyChat(), plan, roUsed: 17, rwUsed: 6, turnsUsed: 7 }
    const h = headerStatus(INFO, s)
    expect(h.complexity).toBe('complex')
    expect(h.phase).toEqual({ number: 2, total: 3, name: 'Drive' })
    expect(h.chips).toEqual([
      { key: 'ro', label: 'sensors 8', level: 'ok' },
      { key: 'rw', label: 'rw 5 / 5', level: 'full' },
      { key: 'turns', label: 'turns 3 / 20', level: 'ok' },
    ])
  })

  it('shows the instruction totals without caps once every phase is completed', () => {
    const plan = parsePlan({ ...PLAN_PAYLOAD, active_phase: null, phases: [phase(0, 'A', 'g', 'done', [1, 5], [3, 1, 2])] })
    const h = headerStatus(INFO, { ...emptyChat(), plan, roUsed: 3, rwUsed: 1, turnsUsed: 2 })
    expect(h.phase).toBeNull()
    expect(h.chips.map((c) => c.label)).toEqual(['sensors 3', 'rw 1', 'turns 2'])
  })
})
describe('composerBlockReason', () => {
  it('returns null when sending is allowed', () => {
    expect(composerBlockReason({ busy: false, connected: true, batteryCutoff: false, batteryMessage: null })).toBeNull()
  })
  it('explains each block', () => {
    expect(composerBlockReason({ busy: true, connected: true, batteryCutoff: false, batteryMessage: null })).toMatch(/working/i)
    expect(composerBlockReason({ busy: false, connected: false, batteryCutoff: false, batteryMessage: null })).toMatch(/agent/i)
    expect(composerBlockReason({ busy: false, connected: true, batteryCutoff: true, batteryMessage: 'battery below cut-off: 8.2 V' })).toBe('battery below cut-off: 8.2 V')
  })
  it('battery wins over busy', () => {
    expect(composerBlockReason({ busy: true, connected: true, batteryCutoff: true, batteryMessage: 'low' })).toBe('low')
  })
})

const text = (seq: number) => ev(seq, 'assistant_text', { text: `t${seq}` })
const seqs = (s: ChatState) => s.items.map((i) => i.seq)

describe('history paging', () => {
  it('history frame records the paging cursor and has_more', () => {
    const s = applyFrame(emptyChat(), { type: 'history', events: [text(5), text(6)], has_more: true })
    expect(s.firstSeq).toBe(5)
    expect(s.hasMore).toBe(true)
    expect(s.firstItemIndex).toBe(FIRST_ITEM_INDEX)
  })

  it('empty history has no more and no cursor', () => {
    const s = applyFrame(emptyChat(), { type: 'history', events: [] })
    expect(s.firstSeq).toBeNull()
    expect(s.hasMore).toBe(false)
  })

  it('prependHistory puts older items first, lowers firstItemIndex and the cursor', () => {
    const start = applyFrame(emptyChat(), { type: 'history', events: [text(5), text(6)], has_more: true })
    const s = prependHistory(start, [text(3), text(4)], false)
    expect(seqs(s)).toEqual([3, 4, 5, 6])
    expect(s.firstSeq).toBe(3)
    expect(s.hasMore).toBe(false)
    expect(s.firstItemIndex).toBe(FIRST_ITEM_INDEX - 2)
    expect(s.lastSeq).toBe(6)
  })

  it('prependHistory dedupes events at or above the cursor', () => {
    const start = applyFrame(emptyChat(), { type: 'history', events: [text(5), text(6)], has_more: true })
    const s = prependHistory(start, [text(4), text(5), text(6)], true)
    expect(seqs(s)).toEqual([4, 5, 6])
    expect(s.firstItemIndex).toBe(FIRST_ITEM_INDEX - 1)
  })

  it('prependHistory with only duplicates changes no items', () => {
    const start = applyFrame(emptyChat(), { type: 'history', events: [text(5)], has_more: true })
    const s = prependHistory(start, [text(5)], false)
    expect(seqs(s)).toEqual([5])
    expect(s.hasMore).toBe(false)
  })

  it('pairs a call on the older page with the standalone result on the newer page', () => {
    const result = ev(11, 'tool_result', { id: 'c1', is_error: false, content: [{ type: 'text', text: 'ok' }], truncated: false })
    const start = applyFrame(emptyChat(), { type: 'history', events: [result, text(12)], has_more: true })
    expect(start.items[0]).toMatchObject({ orphan: true })
    const call = ev(10, 'tool_call', { id: 'c1', name: 'look', kind: 'sensor', input: {} })
    const s = prependHistory(start, [call], false)
    expect(s.items).toHaveLength(2)
    expect(s.items[0]).toMatchObject({ kind: 'tool', id: 'c1', name: 'look', result: { isError: false } })
    expect(s.items[0]).not.toHaveProperty('orphan', true)
    expect(s.firstItemIndex).toBe(FIRST_ITEM_INDEX)
  })

  it('pairs call and result inside the older page and leaves a still-unmatched result standalone', () => {
    const start = applyFrame(emptyChat(), { type: 'history', events: [text(20)], has_more: true })
    const page = [
      ev(10, 'tool_result', { id: 'old', is_error: false, content: [], truncated: false }),
      ev(11, 'tool_call', { id: 'c', name: 'a', kind: 'effector', input: {} }),
      ev(12, 'tool_result', { id: 'c', is_error: false, content: [], truncated: false }),
    ]
    const s = prependHistory(start, page, true)
    expect(s.items.map((i) => (i.kind === 'tool' ? [i.id, i.orphan === true, i.result !== null] : null))).toEqual([
      ['old', true, true],
      ['c', false, true],
      null,
    ])
  })
})

describe('trimWindow', () => {
  const many = (n: number) => run(Array.from({ length: n }, (_, i) => text(i + 1)))

  it('keeps everything within the window', () => {
    const s = many(10)
    expect(trimWindow(s, 10)).toBe(s)
  })

  it('drops the oldest items, marks has_more, moves cursor and raises firstItemIndex', () => {
    const s = trimWindow(many(15), 10)
    expect(s.items).toHaveLength(10)
    expect(s.items[0].seq).toBe(6)
    expect(s.firstSeq).toBe(6)
    expect(s.hasMore).toBe(true)
    expect(s.firstItemIndex).toBe(FIRST_ITEM_INDEX + 5)
    expect(s.lastSeq).toBe(15)
  })

  it('defaults to MAX_ITEMS', () => {
    expect(MAX_ITEMS).toBe(400)
    expect(trimWindow(many(MAX_ITEMS + 1)).items).toHaveLength(MAX_ITEMS)
  })

  it('applyFrame trims live events only when asked to', () => {
    let s = many(10)
    s = applyFrame(s, text(11), 10)
    expect(s.items).toHaveLength(10)
    expect(s.items[0].seq).toBe(2)
    s = applyFrame(s, text(12), null)
    expect(s.items).toHaveLength(11)
  })
})

describe('historyQuery', () => {
  it('builds the paging URL', () => {
    expect(historyQuery(42, 100)).toBe('/api/agent/history?before_seq=42&limit=100')
    expect(historyQuery(null, 50)).toBe('/api/agent/history?limit=50')
  })
})
