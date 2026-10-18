// @vitest-environment jsdom
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { cleanup, fireEvent, render, screen, waitFor } from '@testing-library/react'
import { DeleteMarkedDialog } from './DeleteMarkedDialog'
import { clearPois } from './api'
import type { Poi } from './types'

vi.mock('./api', () => ({ clearPois: vi.fn() }))

function mk(id: string, over: Partial<Poi>): Poi {
  return {
    id, kind: 'point', frame: 'map', x: 0, y: 0, polygon: [], radius_m: 0.5, name: id, note: '',
    status: 'open', created_by: 'user', created_at: 1, updated_at: 1, ...over,
  }
}
const area: [number, number][] = [[0, 0], [1, 0], [1, 1]]
const POIS: Poi[] = [
  mk('t1', { note: '#target' }),
  mk('t2', { note: '#target' }),
  mk('t3', { note: '#target' }),
  mk('s1', { kind: 'area', zone: 'slow', speed_limit_pct: 50, polygon: area }),
  mk('s2', { kind: 'area', zone: 'slow', speed_limit_pct: 50, polygon: area }),
  mk('k1', { kind: 'area', zone: 'keepout', polygon: area }),
  mk('k2', { kind: 'area', zone: 'keepout', note: '#target', polygon: area }),
]

function setup(marked: string[], pois = POIS) {
  const onDone = vi.fn()
  const onClose = vi.fn()
  render(<DeleteMarkedDialog open tabId="map" pois={pois} marked={new Set(marked)} onDone={onDone} onClose={onClose} />)
  return { onDone, onClose }
}
const confirmBtn = () => screen.getByRole('button', { name: 'Delete' }) as HTMLButtonElement

beforeEach(() => {
  vi.mocked(clearPois).mockReset()
})
afterEach(() => cleanup())

describe('DeleteMarkedDialog', () => {
  it('names the count and each category with its count', () => {
    setup(['t1', 't2', 't3', 's1', 's2'])
    expect(screen.getByText('Delete 5 POIs?')).toBeTruthy()
    expect(screen.getByTestId('delete-marked-categories').textContent).toBe('3 target, 2 slow')
  })

  it('without keep-outs confirm posts exactly the marked ids', async () => {
    vi.mocked(clearPois).mockResolvedValue({ ok: true, message: 'x', poi: null, removed: 2 })
    const { onDone } = setup(['t1', 's1'])
    expect(screen.queryByRole('checkbox')).toBeNull()
    fireEvent.click(confirmBtn())
    await waitFor(() => expect(screen.getByText('Removed 2 POIs')).toBeTruthy())
    expect(clearPois).toHaveBeenCalledWith('map', { ids: ['t1', 's1'] })
    expect(onDone).toHaveBeenCalled()
  })

  it('with a keep-out confirm stays disabled until the extra checkbox is ticked', () => {
    setup(['t1', 'k1'])
    expect(screen.getByRole('alert').textContent).toContain('k1')
    const box = screen.getByRole('checkbox', { name: 'I understand this removes 1 keep-out zone(s)' }) as HTMLInputElement
    expect(box.checked).toBe(false)
    expect(confirmBtn().disabled).toBe(true)
    fireEvent.click(box)
    expect(confirmBtn().disabled).toBe(false)
  })

  it('catches a keep-out zone that carries another tag', () => {
    setup(['k2'])
    expect(screen.getByRole('checkbox', { name: 'I understand this removes 1 keep-out zone(s)' })).toBeTruthy()
    expect(confirmBtn().disabled).toBe(true)
  })

  it('shows a clearPois error and keeps the marks', async () => {
    vi.mocked(clearPois).mockResolvedValue({ ok: false, message: 'store down', poi: null })
    const { onDone } = setup(['t1'])
    fireEvent.click(confirmBtn())
    await waitFor(() => expect(screen.getByRole('alert').textContent).toContain('store down'))
    expect(onDone).not.toHaveBeenCalled()
  })

  it('disables confirm when nothing is marked', () => {
    setup([])
    expect(confirmBtn().disabled).toBe(true)
  })
})
