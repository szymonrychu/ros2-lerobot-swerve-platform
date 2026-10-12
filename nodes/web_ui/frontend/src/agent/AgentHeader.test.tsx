// @vitest-environment jsdom
import { afterEach, describe, expect, it, vi } from 'vitest'
import { cleanup, fireEvent, render, screen } from '@testing-library/react'
import { AgentHeader } from './AgentHeader'
import { emptyChat, parsePlan } from './agentModel'
import type { AgentInfo, ChatState } from './agentModel'
import { designViolations, SOURCES } from '../layout/designRules'

afterEach(cleanup)

const INFO = { busy: false, model: 'claude-test-1', max_turns: 10, ro_used: 1, rw_used: 2, turns_used: 3 } as AgentInfo

function chatWithPlan(busy: boolean): ChatState {
  const plan = parsePlan({
    complexity: 'simple',
    phases: [{ index: 0, name: 'scout', goal: 'look', rw_cap: 3, turn_cap: 5, status: 'active' }],
    active_phase: 0,
    time_budget_min: 30,
    time_left_s: 600,
    time_left_at: Date.now() / 1000,
  })
  return { ...emptyChat(), busy, plan }
}

function renderHeader(over: Partial<Parameters<typeof AgentHeader>[0]> = {}) {
  const props = { info: INFO, chat: chatWithPlan(true), connected: true, resetBlocked: false, onStop: vi.fn(), onReset: vi.fn(), ...over }
  return { props, ...render(<AgentHeader {...props} />) }
}

describe('AgentHeader', () => {
  it('shows state, phase dropdown and time left, with no Chip', () => {
    const { container } = renderHeader()
    expect(screen.getByTestId('agent-state').textContent).toContain('working')
    expect(screen.getByTestId('active-phase')).toBeTruthy()
    expect(screen.getByTestId('time-budget').textContent).toContain('budget 30 min')
    expect(container.querySelector('.MuiChip-root')).toBeNull()
    expect(SOURCES['agent/AgentHeader.tsx']).not.toContain('<Chip')
  })

  it('shows the busy state once, with no progress bar', () => {
    const { container } = renderHeader()
    expect(container.textContent?.match(/working/g)).toHaveLength(1)
    expect(screen.queryByRole('progressbar')).toBeNull()
  })

  it('keeps model and usage in the state tooltip', () => {
    renderHeader()
    const label = screen.getByTestId('agent-state').querySelector('[aria-label]')?.getAttribute('aria-label') ?? ''
    expect(label).toContain('claude-test-1')
    expect(label).toContain('sensors')
    expect(label).toContain('rw')
    expect(label).toContain('turns')
  })

  it('shows idle and disconnected', () => {
    renderHeader({ chat: { ...emptyChat(), busy: false } })
    expect(screen.getByTestId('agent-state').textContent).toContain('idle')
    cleanup()
    renderHeader({ chat: { ...emptyChat(), busy: false }, connected: false })
    expect(screen.getByTestId('agent-state').textContent).toContain('disconnected')
  })

  it('stops the agent', () => {
    const { props } = renderHeader()
    fireEvent.click(screen.getByRole('button', { name: 'Stop agent' }))
    expect(props.onStop).toHaveBeenCalledTimes(1)
  })

  it('asks before resetting the session', () => {
    const { props } = renderHeader({ chat: { ...emptyChat(), busy: false } })
    fireEvent.click(screen.getByRole('button', { name: 'New session' }))
    expect(props.onReset).not.toHaveBeenCalled()
    expect(screen.getByText('Start a new session?')).toBeTruthy()
    fireEvent.click(screen.getByRole('dialog').querySelector('button.MuiButton-outlined') as HTMLElement)
    expect(props.onReset).toHaveBeenCalledTimes(1)
  })

  it('cancel does not reset', () => {
    const { props } = renderHeader({ chat: { ...emptyChat(), busy: false } })
    fireEvent.click(screen.getByRole('button', { name: 'New session' }))
    fireEvent.click(screen.getByRole('button', { name: 'Cancel' }))
    expect(props.onReset).not.toHaveBeenCalled()
  })

  it('disables New session while blocked', () => {
    renderHeader({ resetBlocked: true })
    expect((screen.getByRole('button', { name: 'New session' }) as HTMLButtonElement).disabled).toBe(true)
  })

  it('follows the design rules in the agent header files', () => {
    for (const file of ['agent/AgentHeader.tsx', 'agent/PhasePanel.tsx', 'agent/PhaseProgress.tsx']) {
      expect(SOURCES[file], file).toBeTruthy()
      expect(designViolations(file, SOURCES[file])).toEqual([])
    }
  })
})
