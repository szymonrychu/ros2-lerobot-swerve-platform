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
  parseAgentFrame,
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

  it('state events update busy and effector calls without adding items', () => {
    const s = run([ev(1, 'state', { busy: true, effector_calls_used: 4 })])
    expect(s.items).toHaveLength(0)
    expect(s.busy).toBe(true)
    expect(s.effectorCallsUsed).toBe(4)
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
    const s = applyFrame(emptyChat(), { type: 'history', events: [ev(1, 'state', { busy: true, effector_calls_used: 1 })] })
    expect(s.busy).toBe(true)
    expect(s.effectorCallsUsed).toBe(1)
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

describe('applyStateSnapshot', () => {
  it('copies busy and effector calls', () => {
    const s = applyStateSnapshot(emptyChat(), {
      busy: true, model: 'm', max_turns: 30, effector_call_cap: 20, effector_calls_used: 3, session_started_at: 1,
    })
    expect(s.busy).toBe(true)
    expect(s.effectorCallsUsed).toBe(3)
  })
})

describe('statusLabel / headerStatus', () => {
  it('maps turn statuses', () => {
    expect(statusLabel('done')).toEqual({ label: 'Done', color: 'success' })
    expect(statusLabel('interrupted')).toEqual({ label: 'Interrupted', color: 'warning' })
    expect(statusLabel('max_turns')).toEqual({ label: 'Turn cap reached', color: 'warning' })
    expect(statusLabel('error')).toEqual({ label: 'Error', color: 'error' })
    expect(statusLabel('weird')).toEqual({ label: 'weird', color: 'default' })
  })

  it('combines info and live state', () => {
    const info = { busy: false, model: 'opus', max_turns: 30, effector_call_cap: 20, effector_calls_used: 0, session_started_at: 1 }
    const s = { ...emptyChat(), busy: true, effectorCallsUsed: 5 }
    expect(headerStatus(info, s)).toEqual({ model: 'opus', busy: true, effectorLabel: '5 / 20', maxTurns: 30 })
    expect(headerStatus(null, s)).toEqual({ model: null, busy: true, effectorLabel: '5', maxTurns: null })
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
