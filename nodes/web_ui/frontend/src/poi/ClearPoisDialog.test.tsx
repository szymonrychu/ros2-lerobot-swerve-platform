// @vitest-environment jsdom
import { afterEach, describe, expect, it, vi } from 'vitest'
import { cleanup, fireEvent, render, screen, waitFor } from '@testing-library/react'
import { ClearPoisDialog, previewCount } from './ClearPoisDialog'
import type { Poi } from './types'

function mk(id: string, over: Partial<Poi>): Poi {
  return {
    id, kind: 'point', frame: 'map', x: 0, y: 0, polygon: [], radius_m: 0.5, name: id, note: '',
    status: 'open', created_by: 'user', created_at: 1, updated_at: 1, ...over,
  }
}

const POIS: Poi[] = [
  mk('p-user', {}),
  mk('p-agent', { created_by: 'agent' }),
  mk('o-agent', { kind: 'object', created_by: 'agent' }),
  mk('a-plain', { kind: 'area', polygon: [[0, 0], [1, 0], [1, 1]] }),
  mk('a-keep', { kind: 'area', zone: 'keepout', polygon: [[0, 0], [1, 0], [1, 1]] }),
  mk('a-slow', { kind: 'area', zone: 'slow', speed_limit_pct: 50, created_by: 'agent', polygon: [[0, 0], [1, 0], [1, 1]] }),
]

function okFetch(removed: number) {
  return vi.fn(async () => new Response(JSON.stringify({ ok: true, message: 'cleared', poi: { removed } }), { status: 200 }))
}

afterEach(() => cleanup())

describe('previewCount', () => {
  it('keeps zones by default and applies creator and kinds', () => {
    const all = { createdBy: null, kinds: ['point', 'area', 'object'] as const, keepZones: true }
    expect(previewCount(POIS, { ...all, kinds: [...all.kinds] })).toBe(4)
    expect(previewCount(POIS, { ...all, kinds: [...all.kinds], keepZones: false })).toBe(6)
    expect(previewCount(POIS, { createdBy: 'agent', kinds: ['point', 'area', 'object'], keepZones: true })).toBe(2)
    expect(previewCount(POIS, { createdBy: 'user', kinds: ['point'], keepZones: true })).toBe(1)
    expect(previewCount(POIS, { createdBy: null, kinds: [], keepZones: true })).toBe(0)
  })
})

describe('ClearPoisDialog', () => {
  it('shows the preview and posts the default clear body', async () => {
    const fetchImpl = okFetch(4)
    const onClose = vi.fn()
    render(<ClearPoisDialog open tabId="map" pois={POIS} onClose={onClose} fetchImpl={fetchImpl as unknown as typeof fetch} />)
    expect(screen.getByTestId('clear-pois-preview').textContent).toContain('4')
    fireEvent.click(screen.getByRole('button', { name: 'Clear' }))
    await waitFor(() => expect(screen.getByText('Removed 4 POIs')).toBeTruthy())
    const [url, init] = fetchImpl.mock.calls[0] as unknown as [string, RequestInit]
    expect(url).toBe('/api/poi?tab=map')
    expect(JSON.parse(String(init.body))).toEqual({ op: 'clear', created_by: null, kinds: ['point', 'area', 'object'], keep_zones: true })
  })

  it('posts creator, kinds and keep_zones per choice', async () => {
    const fetchImpl = okFetch(1)
    render(<ClearPoisDialog open tabId="map" pois={POIS} onClose={vi.fn()} fetchImpl={fetchImpl as unknown as typeof fetch} />)
    fireEvent.click(screen.getByLabelText("Agent's"))
    fireEvent.click(screen.getByLabelText('object'))
    fireEvent.click(screen.getByLabelText('Keep keep-out and slow zones'))
    expect(screen.getByTestId('clear-pois-preview').textContent).toContain('3')
    fireEvent.click(screen.getByRole('button', { name: 'Clear' }))
    await waitFor(() => expect(fetchImpl).toHaveBeenCalled())
    const [, init] = fetchImpl.mock.calls[0] as unknown as [string, RequestInit]
    expect(JSON.parse(String(init.body))).toEqual({ op: 'clear', created_by: 'agent', kinds: ['point', 'area'], keep_zones: false })
  })

  it('disables confirm when nothing matches', () => {
    render(<ClearPoisDialog open tabId="map" pois={[]} onClose={vi.fn()} />)
    expect((screen.getByRole('button', { name: 'Clear' }) as HTMLButtonElement).disabled).toBe(true)
  })

  it('shows the error of a failed clear', async () => {
    const fetchImpl = vi.fn(async () => new Response(JSON.stringify({ ok: false, message: 'store down' }), { status: 503 }))
    render(<ClearPoisDialog open tabId="map" pois={POIS} onClose={vi.fn()} fetchImpl={fetchImpl as unknown as typeof fetch} />)
    fireEvent.click(screen.getByRole('button', { name: 'Clear' }))
    await waitFor(() => expect(screen.getByText('store down')).toBeTruthy())
  })

  it('renders the choose-individually link only when the callback is given', () => {
    const { rerender } = render(<ClearPoisDialog open tabId="map" pois={POIS} onClose={vi.fn()} />)
    expect(screen.queryByText('Choose individual POIs')).toBeNull()
    const onChoose = vi.fn()
    const onClose = vi.fn()
    rerender(<ClearPoisDialog open tabId="map" pois={POIS} onClose={onClose} onChooseIndividually={onChoose} />)
    fireEvent.click(screen.getByText('Choose individual POIs'))
    expect(onClose).toHaveBeenCalled()
    expect(onChoose).toHaveBeenCalled()
  })
})
