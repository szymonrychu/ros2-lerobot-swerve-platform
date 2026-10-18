// @vitest-environment jsdom
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { cleanup, fireEvent, render, screen, within } from '@testing-library/react'
import { ThemeProvider } from '@mui/material/styles'
import { theme } from '../theme'
import { PoiListPanel } from './PoiPanels'
import type { Photo } from '../photos/api'
import type { Poi } from './types'

function mk(id: string, note: string, extra: Partial<Poi> = {}): Poi {
  return {
    id, kind: 'point', frame: 'map', x: 0, y: 0, polygon: [], radius_m: 0.5, name: id, note,
    status: 'open', created_by: 'user', created_at: 1, updated_at: 1, ...extra,
  }
}

const POIS = [mk('t1', '#target a'), mk('t2', '#target b'), mk('d1', '#detail c'), mk('plain', '')]

beforeEach(() => {
  window.matchMedia = ((query: string) => ({
    matches: true, media: query, addEventListener: vi.fn(), removeEventListener: vi.fn(), addListener: vi.fn(),
    removeListener: vi.fn(), dispatchEvent: vi.fn(), onchange: null,
  })) as typeof window.matchMedia
})
afterEach(() => cleanup())

function renderList(pois: Poi[] = POIS) {
  const ui = (p: Poi[]) => (
    <ThemeProvider theme={theme}>
      <PoiListPanel pois={p} selectedId={null} open onToggle={vi.fn()} onFocus={vi.fn()} />
    </ThemeProvider>
  )
  const r = render(ui(pois))
  return { rerender: (p: Poi[]) => r.rerender(ui(p)) }
}

const box = (name: string) => within(screen.getByTestId('poi-list-panel')).getByRole('checkbox', { name: `Mark ${name}` }) as HTMLInputElement
const chip = (label: RegExp) => screen.getByRole('button', { name: label })

describe('POI list filter and marking', () => {
  it('filters by one category, hides the others and shows counts', () => {
    renderList()
    expect(chip(/^Target \(2\)/)).toBeTruthy()
    expect(chip(/^Detail \(1\)/)).toBeTruthy()
    expect(chip(/^uncategorised \(1\)/)).toBeTruthy()
    fireEvent.click(chip(/^Target/))
    expect(screen.queryByText('d1')).toBeNull()
    expect(screen.queryByText('plain')).toBeNull()
    expect(screen.getByText('t1')).toBeTruthy()
    fireEvent.click(chip(/^Target/))
    expect(screen.getByText('d1')).toBeTruthy()
  })

  it('a row checkbox marks only that POI', () => {
    renderList()
    fireEvent.click(box('t1'))
    expect(box('t1').checked).toBe(true)
    expect(box('t2').checked).toBe(false)
    expect(screen.getByTestId('poi-marked-count').textContent).toContain('1')
  })

  it('Mark all visible with a filter marks exactly the visible POIs', () => {
    renderList()
    fireEvent.click(chip(/^Target/))
    fireEvent.click(screen.getByRole('button', { name: 'Mark all visible' }))
    expect(box('t1').checked && box('t2').checked).toBe(true)
    fireEvent.click(chip(/^Target/))
    expect(box('d1').checked).toBe(false)
    expect(box('plain').checked).toBe(false)
    expect(screen.getByTestId('poi-marked-count').textContent).toContain('2')
  })

  it('changing the filter unmarks POIs that became hidden', () => {
    renderList()
    fireEvent.click(box('t1'))
    fireEvent.click(box('d1'))
    fireEvent.click(chip(/^Detail/))
    expect(screen.getByTestId('poi-marked-count').textContent).toContain('1')
    fireEvent.click(chip(/^Detail/))
    expect(box('t1').checked).toBe(false)
    expect(box('d1').checked).toBe(true)
  })

  it('a POI deleted elsewhere is pruned from the marks', () => {
    const { rerender } = renderList()
    fireEvent.click(box('t1'))
    rerender(POIS.filter((p) => p.id !== 't1'))
    expect(screen.getByTestId('poi-marked-count').textContent).toContain('0')
  })

  it('Clear marks empties the marks', () => {
    renderList()
    fireEvent.click(screen.getByRole('button', { name: 'Mark all visible' }))
    fireEvent.click(screen.getByRole('button', { name: 'Clear marks' }))
    expect(screen.getByTestId('poi-marked-count').textContent).toContain('0')
    expect(box('t1').checked).toBe(false)
  })
})

describe('Delete marked button', () => {
  function renderWithTab() {
    render(
      <ThemeProvider theme={theme}>
        <PoiListPanel pois={POIS} selectedId={null} open onToggle={vi.fn()} onFocus={vi.fn()} tabId="map" />
      </ThemeProvider>,
    )
  }

  it('is disabled at 0 marked and opens the confirmation with the count', () => {
    renderWithTab()
    expect((screen.getByRole('button', { name: 'Delete marked (0)' }) as HTMLButtonElement).disabled).toBe(true)
    fireEvent.click(screen.getByRole('button', { name: 'Mark all visible' }))
    fireEvent.click(screen.getByRole('button', { name: 'Delete marked (4)' }))
    expect(screen.getByText('Delete 4 POIs?')).toBeTruthy()
  })

  it('a photo row can be marked and counts in Delete marked (N)', () => {
    const photo = { id: 'p1', ts: 1, camera: 'c', tool: 't', name: 'shot', note: '', poiIds: [], x: null, y: null, view: null, bytes: 1, width: 1, height: 1 } as Photo
    render(
      <ThemeProvider theme={theme}>
        <PoiListPanel pois={POIS} photos={[photo]} selectedId={null} open onToggle={vi.fn()} onFocus={vi.fn()} tabId="map" />
      </ThemeProvider>,
    )
    fireEvent.click(box('shot'))
    expect(box('shot').checked).toBe(true)
    fireEvent.click(box('t1'))
    expect(screen.getByRole('button', { name: 'Delete marked (2)' })).toBeTruthy()
    fireEvent.click(screen.getByRole('button', { name: 'Delete marked (2)' }))
    expect(screen.getByText('Delete 1 POIs and 1 photos?')).toBeTruthy()
  })

  const PHOTO = { id: 'p1', ts: 1, camera: 'c', tool: 't', name: 'shot', note: '', poiIds: [], x: null, y: null, view: null, bytes: 1, width: 1, height: 1 } as Photo
  const ui = (photos: Photo[]) => (
    <ThemeProvider theme={theme}>
      <PoiListPanel pois={POIS} photos={photos} selectedId={null} open onToggle={vi.fn()} onFocus={vi.fn()} tabId="map" />
    </ThemeProvider>
  )

  it('Mark all visible also marks photos', () => {
    render(ui([PHOTO]))
    fireEvent.click(screen.getByRole('button', { name: 'Mark all visible' }))
    expect(box('shot').checked).toBe(true)
    expect(screen.getByRole('button', { name: 'Delete marked (5)' })).toBeTruthy()
  })

  it('photo rows are hidden while a category filter is active', () => {
    render(ui([PHOTO]))
    expect(screen.getByText('shot')).toBeTruthy()
    fireEvent.click(chip(/^Target/))
    expect(screen.queryByText('shot')).toBeNull()
    fireEvent.click(screen.getByRole('button', { name: 'Mark all visible' }))
    fireEvent.click(chip(/^Target/))
    expect(box('shot').checked).toBe(false)
  })

  it('a mark is pruned when its photo disappears', () => {
    const r = render(ui([PHOTO]))
    fireEvent.click(box('shot'))
    expect(screen.getByRole('button', { name: 'Delete marked (1)' })).toBeTruthy()
    r.rerender(ui([]))
    expect(screen.getByRole('button', { name: 'Delete marked (0)' })).toBeTruthy()
  })
})
