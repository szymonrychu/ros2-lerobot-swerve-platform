// @vitest-environment jsdom
import { afterEach, describe, expect, it } from 'vitest'
import { cleanup, fireEvent, render, screen } from '@testing-library/react'
import { PhasePanel, PHASE_PANEL_STORAGE_KEY, readPhasePanelOpen, writePhasePanelOpen } from './PhasePanel'
import type { PhaseStorage } from './PhasePanel'
import type { PhaseRow } from './agentModel'

afterEach(cleanup)

const BASE: Omit<PhaseRow, 'number' | 'name' | 'status' | 'active'> = {
  goal: 'goal', summary: '', raised: false, attemptLabel: null, iterationLabel: null, pRetrySuccess: null, sensorCalls: 0, bars: [],
}
const ROWS: PhaseRow[] = [
  { ...BASE, number: 1, name: 'scout', status: 'done', active: false },
  { ...BASE, number: 2, name: 'grasp', status: 'active', active: true },
]
const HEADER = { number: 2, total: 2, name: 'grasp', attemptLabel: 'attempt 1/3', iterationLabel: null }

function memoryStorage(initial: Record<string, string> = {}): PhaseStorage & { data: Record<string, string> } {
  const data = { ...initial }
  return { data, getItem: (k) => data[k] ?? null, setItem: (k, v) => { data[k] = v } }
}
const throwing: PhaseStorage = {
  getItem: () => { throw new Error('denied') },
  setItem: () => { throw new Error('denied') },
}

describe('phase storage helpers', () => {
  it('reads persisted open and closed, default closed', () => {
    expect(readPhasePanelOpen(memoryStorage({ [PHASE_PANEL_STORAGE_KEY]: '1' }))).toBe(true)
    expect(readPhasePanelOpen(memoryStorage({ [PHASE_PANEL_STORAGE_KEY]: '0' }))).toBe(false)
    expect(readPhasePanelOpen(memoryStorage())).toBe(false)
    expect(readPhasePanelOpen(undefined)).toBe(false)
  })
  it('survives a throwing storage', () => {
    expect(readPhasePanelOpen(throwing)).toBe(false)
    expect(() => writePhasePanelOpen(throwing, true)).not.toThrow()
  })
  it('writes the flag', () => {
    const s = memoryStorage()
    writePhasePanelOpen(s, true)
    expect(s.data[PHASE_PANEL_STORAGE_KEY]).toBe('1')
  })
})

describe('PhasePanel', () => {
  it('renders nothing without rows', () => {
    const { container } = render(<PhasePanel rows={[]} header={null} storage={memoryStorage()} />)
    expect(container.innerHTML).toBe('')
  })
  it('is closed by default, chip shows phase n/m and progress', () => {
    render(<PhasePanel rows={ROWS} header={HEADER} storage={memoryStorage()} />)
    expect(screen.getByTestId('active-phase').textContent).toContain('2/2 grasp')
    expect(screen.getByTestId('phase-attempt').textContent).toBe('attempt 1/3')
    expect(screen.queryByTestId('phase-row-1')).toBeNull()
  })
  it('toggles on click and remembers it', () => {
    const s = memoryStorage()
    render(<PhasePanel rows={ROWS} header={HEADER} storage={s} />)
    fireEvent.click(screen.getByTestId('active-phase'))
    expect(screen.getByTestId('phase-panel')).toBeTruthy()
    expect(screen.getByTestId('phase-row-1')).toBeTruthy()
    expect(screen.getByTestId('phase-row-2')).toBeTruthy()
    expect(s.data[PHASE_PANEL_STORAGE_KEY]).toBe('1')
    fireEvent.click(screen.getByTestId('active-phase'))
    expect(s.data[PHASE_PANEL_STORAGE_KEY]).toBe('0')
  })
  it('opens from persisted open', () => {
    render(<PhasePanel rows={ROWS} header={HEADER} storage={memoryStorage({ [PHASE_PANEL_STORAGE_KEY]: '1' })} />)
    expect(screen.getByTestId('phase-row-2')).toBeTruthy()
  })
  it('renders closed with a throwing storage and still toggles', () => {
    render(<PhasePanel rows={ROWS} header={HEADER} storage={throwing} />)
    expect(screen.queryByTestId('phase-row-1')).toBeNull()
    fireEvent.click(screen.getByTestId('active-phase'))
    expect(screen.getByTestId('phase-row-1')).toBeTruthy()
  })
  it('shows row status as text, coloured only for failed and skipped', () => {
    const rows: PhaseRow[] = [
      { ...BASE, number: 1, name: 'a', status: 'done', active: false },
      { ...BASE, number: 2, name: 'b', status: 'failed', active: false },
      { ...BASE, number: 3, name: 'c', status: 'skipped', active: false },
    ]
    render(<PhasePanel rows={rows} header={null} storage={memoryStorage({ [PHASE_PANEL_STORAGE_KEY]: '1' })} />)
    expect(document.querySelector('.MuiChip-root')).toBeNull()
    const color = (n: number) => getComputedStyle(screen.getByTestId(`phase-row-${n}`).querySelector('[data-testid="phase-status"]') as HTMLElement).color
    expect(screen.getByTestId('phase-row-1').textContent).toContain('done')
    const neutral = color(1)
    expect(color(2)).not.toBe(neutral)
    expect(color(3)).not.toBe(neutral)
    expect(color(2)).not.toBe(color(3))
  })
  it('joins raised, attempt, iteration and p as plain text with a hyphen', () => {
    const rows: PhaseRow[] = [{ ...BASE, number: 1, name: 'a', status: 'active', active: true, raised: true, attemptLabel: 'attempt 2/3', iterationLabel: 'iteration 4', pRetrySuccess: 0.7 }]
    render(<PhasePanel rows={rows} header={null} storage={memoryStorage({ [PHASE_PANEL_STORAGE_KEY]: '1' })} />)
    expect(screen.getByTestId('phase-progress').textContent).toBe('raised - attempt 2/3 - iteration 4 - p 0.70')
  })
})
