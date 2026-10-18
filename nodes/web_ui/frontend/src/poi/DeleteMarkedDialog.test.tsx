// @vitest-environment jsdom
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { cleanup, fireEvent, render, screen, waitFor } from '@testing-library/react'
import { DeleteMarkedDialog } from './DeleteMarkedDialog'
import { clearPois } from './api'
import type { Photo } from '../photos/api'
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

function ph(id: string): Photo {
  return {
    id, ts: 1, camera: 'c', tool: 't', name: id, note: '', poiIds: [], x: null, y: null, view: null, bytes: 1, width: 1, height: 1,
  } as Photo
}
const PHOTOS = [ph('p1'), ph('p2')]

function setup(marked: string[], pois = POIS, photos: Photo[] = []) {
  const onDone = vi.fn()
  const onClose = vi.fn()
  render(<DeleteMarkedDialog open tabId="map" pois={pois} photos={photos} marked={new Set(marked)} onDone={onDone} onClose={onClose} />)
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

  it('keeps the confirmation text after success when the parent clears the marks', async () => {
    vi.mocked(clearPois).mockResolvedValue({ ok: true, message: 'x', poi: null, removed: 2 })
    const onDone = vi.fn()
    const props = { open: true, tabId: 'map', pois: POIS, onDone, onClose: vi.fn() }
    const { rerender } = render(<DeleteMarkedDialog {...props} marked={new Set(['t1', 'k1'])} />)
    fireEvent.click(screen.getByRole('checkbox'))
    fireEvent.click(confirmBtn())
    await waitFor(() => expect(screen.getByText('Removed 2 POIs')).toBeTruthy())
    rerender(<DeleteMarkedDialog {...props} marked={new Set()} />)
    expect(screen.getByText('Delete 2 POIs?')).toBeTruthy()
    expect(screen.getByTestId('delete-marked-categories').textContent).toBe('1 target, 1 keepout')
    expect(screen.getByRole('alert').textContent).toContain('k1')
  })

  it('counts marked photos in the summary and posts their ids with the POI ids', async () => {
    vi.mocked(clearPois).mockResolvedValue({ ok: true, message: 'x', poi: null, removed: 1, pois: 1, photos: 2 })
    setup(['t1', 'p1', 'p2'], POIS, PHOTOS)
    expect(screen.getByText('Delete 1 POIs and 2 photos?')).toBeTruthy()
    expect(screen.getByTestId('delete-marked-categories').textContent).toBe('1 target')
    fireEvent.click(confirmBtn())
    await waitFor(() => expect(screen.getByText('Removed 1 POIs and 2 photos')).toBeTruthy())
    expect(clearPois).toHaveBeenCalledWith('map', { ids: ['t1', 'p1', 'p2'] })
  })

  it('only photos marked still enables confirm', () => {
    setup(['p1'], POIS, PHOTOS)
    expect(screen.getByText('Delete 0 POIs and 1 photos?')).toBeTruthy()
    expect(confirmBtn().disabled).toBe(false)
  })
})
