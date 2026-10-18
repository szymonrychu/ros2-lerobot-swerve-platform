// @vitest-environment jsdom
import { cleanup, fireEvent, render, screen, waitFor } from '@testing-library/react'
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import RetrosTab from './RetrosTab'
import * as api from '../agent/agentApi'
import type { Retrospective } from '../agent/agentApi'

// A plain function, not a vi.fn: vitest's spy reports a rejecting call as an unhandled error even when the component catches it.
type Fetch = typeof api.fetchRetrospectives
const hoisted = vi.hoisted(() => ({ impl: null as unknown as Fetch, calls: [] as unknown[][] }))
vi.mock('../agent/agentApi', async (orig) => ({
  ...(await orig<typeof api>()),
  fetchRetrospectives: (...args: unknown[]) => {
    hoisted.calls.push(args)
    return hoisted.impl(...(args as Parameters<Fetch>))
  },
}))
const fetchMock = {
  resolve: (...pages: api.RetrospectivesPage[]) => {
    let i = 0
    hoisted.impl = async () => pages[Math.min(i++, pages.length - 1)]
  },
  reject: (message: string) => {
    hoisted.impl = () => Promise.reject(new Error(message))
  },
  get calls() {
    return hoisted.calls
  },
}

function retro(over: Partial<Retrospective>): Retrospective {
  return {
    id: 'a', created_at: 1700000000, instruction: 'go', workflow: 'survey', outcome: 'done', trivial: false,
    author: 'agent', summary: 'Surveyed the hall', went_well: ['mapped fast'], went_wrong: ['door stuck'],
    improvements: ['retry doors'], notes_for_fixers: ['check lidar'], skipped_mandatory: ['photo_step'],
    phases: [{ name: 'scan', outcome: 'done' }], cost_usd: 0.1234, num_turns: 12, effector_calls: 3, duration_s: 75,
    ...over,
  }
}
const tab = { id: 'r', type: 'agent_retros', label: 'Retros' }
const props = { tab, topicData: {}, publish: vi.fn() }

beforeEach(() => {
  hoisted.calls.length = 0
})
afterEach(() => {
  cleanup()
  vi.useRealTimers()
})

describe('RetrosTab', () => {
  it('renders a list from a mocked fetch', async () => {
    fetchMock.resolve({
      retrospectives: [retro({ id: 'a' }), retro({ id: 'b', workflow: null, outcome: 'failed', author: 'runner', summary: 'Free job broke' })],
      total: 2,
    })
    render(<RetrosTab {...props} />)
    expect(await screen.findByText('Surveyed the hall')).toBeTruthy()
    expect(screen.getByText('Free job broke')).toBeTruthy()
    expect(screen.getByText('free')).toBeTruthy()
    expect(screen.getByText('survey')).toBeTruthy()
    expect(screen.getByText('failed')).toBeTruthy()
    expect(screen.getAllByText('runner')).toHaveLength(1)
  })

  it('expands a row to show sections and skipped steps', async () => {
    fetchMock.resolve({ retrospectives: [retro({})], total: 1 })
    render(<RetrosTab {...props} />)
    fireEvent.click(await screen.findByText('Surveyed the hall'))
    for (const text of ['mapped fast', 'door stuck', 'retry doors', 'check lidar', 'photo_step', 'scan']) {
      expect(await screen.findByText(new RegExp(text))).toBeTruthy()
    }
    expect(screen.getByText(/Skipped mandatory/i)).toBeTruthy()
    expect(screen.getByText(/12 turns/)).toBeTruthy()
  })

  it('shows the empty state', async () => {
    fetchMock.resolve({ retrospectives: [], total: 0 })
    render(<RetrosTab {...props} />)
    expect(await screen.findByText('No retrospectives yet')).toBeTruthy()
  })

  it('shows the error message', async () => {
    fetchMock.reject('claude_agent unreachable')
    render(<RetrosTab {...props} />)
    expect(await screen.findByText(/claude_agent unreachable/)).toBeTruthy()
  })

  it('Load older passes before', async () => {
    fetchMock.resolve(
      { retrospectives: [retro({ id: 'a', created_at: 500 })], total: 2 },
      { retrospectives: [retro({ id: 'b', created_at: 400, summary: 'Older one' })], total: 2 },
    )
    render(<RetrosTab {...props} />)
    fireEvent.click(await screen.findByRole('button', { name: /load older/i }))
    expect(await screen.findByText('Older one')).toBeTruthy()
    expect(fetchMock.calls[fetchMock.calls.length - 1]).toEqual([expect.any(Number), 500])
    await waitFor(() => expect(screen.queryByRole('button', { name: /load older/i })).toBeNull())
  })

  it('refreshes on the button and every 30 s', async () => {
    vi.useFakeTimers()
    fetchMock.resolve({ retrospectives: [retro({})], total: 1 })
    render(<RetrosTab {...props} />)
    await vi.advanceTimersByTimeAsync(0)
    expect(fetchMock.calls).toHaveLength(1)
    await vi.advanceTimersByTimeAsync(30000)
    expect(fetchMock.calls).toHaveLength(2)
    fireEvent.click(screen.getByRole('button', { name: /refresh/i }))
    expect(fetchMock.calls).toHaveLength(3)
  })
})
