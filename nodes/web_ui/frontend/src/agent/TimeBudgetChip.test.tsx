// @vitest-environment jsdom
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { act, cleanup, render, screen } from '@testing-library/react'
import { TimeBudgetChip } from './TimeBudgetChip'
import { emptyChat, parsePlan } from './agentModel'
import type { ChatState } from './agentModel'

const T0 = 1_700_000_000

function chat(busy: boolean): ChatState {
  const plan = parsePlan({ complexity: 'simple', phases: [], time_budget_min: 30, time_left_s: 600, time_left_at: T0 })
  return { ...emptyChat(), busy, plan }
}

describe('TimeBudgetChip', () => {
  beforeEach(() => {
    vi.useFakeTimers()
    vi.setSystemTime(T0 * 1000)
  })
  afterEach(() => {
    cleanup()
    vi.useRealTimers()
  })

  it('renders nothing before a plan declares a budget', () => {
    const { container } = render(<TimeBudgetChip state={emptyChat()} />)
    expect(container.innerHTML).toBe('')
  })

  it('shows the declared budget and the time left, ticking every second while the job runs', () => {
    render(<TimeBudgetChip state={chat(true)} />)
    expect(screen.getByTestId('time-budget').textContent).toBe('budget 30 min, 10:00 left')
    act(() => {
      vi.advanceTimersByTime(1000)
    })
    expect(screen.getByTestId('time-budget').textContent).toBe('budget 30 min, 9:59 left')
  })

  it('stops ticking when the job is not running', () => {
    render(<TimeBudgetChip state={chat(false)} />)
    act(() => {
      vi.advanceTimersByTime(5000)
    })
    expect(screen.getByTestId('time-budget').textContent).toBe('budget 30 min, 10:00 left')
  })
})
