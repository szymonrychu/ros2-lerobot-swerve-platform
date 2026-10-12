// @vitest-environment jsdom
import { afterEach, describe, expect, it } from 'vitest'
import { cleanup, render, screen } from '@testing-library/react'
import { ChatEntry } from './ChatLog'
import type { ChatItem } from './agentModel'
import { designViolations, SOURCES } from '../layout/designRules'

afterEach(cleanup)

const noop = (): void => undefined

const TOOL: ChatItem = { key: 't1', kind: 'tool', seq: 1, id: 'c1', name: 'move_arm', fullName: 'robot.move_arm', toolKind: 'effector', input: { x: 1 }, result: null }
const PLAN: ChatItem = {
  key: 'p1',
  kind: 'plan',
  seq: 2,
  complexity: 'multi_step',
  rationale: 'because',
  phases: [{ index: 0, name: 'scout', goal: 'look', rwCap: 3, turnCap: 5, status: 'pending' }],
} as unknown as ChatItem
const STARTED: ChatItem = { key: 's1', kind: 'phase_started', seq: 3, index: 0, name: 'scout', goal: 'look', rwCap: 3, turnCap: 5, attempt: 1, maxAttempts: null, iteration: 0 } as unknown as ChatItem
const COMPLETED: ChatItem = {
  key: 'c1',
  kind: 'phase_completed',
  seq: 4,
  index: 0,
  name: 'scout',
  goal: 'look',
  outcome: 'done',
  summary: 'ok',
  usage: { roUsed: 1, rwUsed: 1, turnsUsed: 1 },
  caps: { rwCap: 3, turnCap: 5 },
  attempt: 1,
  maxAttempts: null,
  iteration: 0,
  pRetrySuccess: null,
} as unknown as ChatItem

function row(item: ChatItem) {
  return render(<ChatEntry item={item} onOpenImage={noop} />)
}

describe('ChatLog', () => {
  it('renders user and agent entries as log rows with a role column and no bubble', () => {
    row({ key: 'u', kind: 'user', seq: 0, text: 'hello robot' })
    expect(screen.getByTestId('chat-role').textContent).toBe('User')
    expect(screen.getByTestId('chat-row').textContent).toContain('hello robot')
    cleanup()
    row({ key: 'a', kind: 'assistant', seq: 0, text: 'on it' })
    expect(screen.getByTestId('chat-role').textContent).toBe('Agent')
  })

  it('labels tool and system rows', () => {
    row(TOOL)
    expect(screen.getByTestId('chat-role').textContent).toBe('Tool')
    cleanup()
    row({ key: 'e', kind: 'error', seq: 0, message: 'boom' })
    expect(screen.getByTestId('chat-role').textContent).toBe('System')
  })

  it('shows the tool kind as plain mono text and colours status only on error', () => {
    const { container } = row(TOOL)
    expect(screen.getByTestId('tool-kind').textContent).toBe('effector')
    expect(container.querySelector('.MuiChip-root')).toBeNull()
    expect(screen.getByTestId('tool-status').getAttribute('data-error')).toBe('false')
    cleanup()
    row({ ...TOOL, result: { isError: true, content: [], truncated: false } } as unknown as ChatItem)
    expect(screen.getByTestId('tool-status').getAttribute('data-error')).toBe('true')
  })

  it('shows complexity only in the plan card, as text, with no chips', () => {
    const { container } = row(PLAN)
    expect(container.textContent).toContain('multi step')
    expect(container.querySelector('.MuiChip-root')).toBeNull()
  })

  it('renders phase start and complete as single chip-free rows', () => {
    const a = row(STARTED)
    expect(screen.getAllByTestId('chat-row')).toHaveLength(1)
    expect(a.container.querySelector('.MuiChip-root')).toBeNull()
    expect(a.container.textContent).toContain('Phase 1 started')
    cleanup()
    const b = row(COMPLETED)
    expect(screen.getAllByTestId('chat-row')).toHaveLength(1)
    expect(b.container.querySelector('.MuiChip-root')).toBeNull()
    expect(b.container.textContent).toContain('Phase 1 done')
  })

  it('has no bubble fill, secondary border or chip in the sources', () => {
    const log = SOURCES['agent/ChatLog.tsx']
    expect(log).not.toContain('primary.dark')
    expect(log).not.toMatch(/borderColor: 'secondary|color="secondary"/)
    expect(log).not.toContain('<Chip')
  })

  it('keeps the composer placeholder short and the empty state one line', () => {
    const tab = SOURCES['tabs/AgentChatTab.tsx']
    expect(tab).not.toMatch(/Enter sends|Shift\+Enter/)
    expect(tab).toContain('placeholder="Message"')
    expect(tab).toContain('No messages yet')
  })

  it('passes the design rules', () => {
    for (const path of ['agent/ChatLog.tsx', 'tabs/AgentChatTab.tsx']) expect(designViolations(path, SOURCES[path])).toEqual([])
  })
})
