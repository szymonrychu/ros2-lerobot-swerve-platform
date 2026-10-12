// @vitest-environment jsdom
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { cleanup, fireEvent, render, screen } from '@testing-library/react'
import { ThemeProvider } from '@mui/material/styles'
import { theme } from '../theme'
import { PoiEditorPanel, PoiListPanel } from './PoiPanels'
import type { Poi } from './types'

let viewportWidth = 1400

/** Minimal matchMedia: evaluates min-width / max-width px conditions against viewportWidth. */
function installMatchMedia(): void {
  window.matchMedia = ((query: string) => {
    const min = /min-width:\s*([\d.]+)px/.exec(query)
    const max = /max-width:\s*([\d.]+)px/.exec(query)
    let matches = true
    if (min && viewportWidth < Number(min[1])) matches = false
    if (max && viewportWidth > Number(max[1])) matches = false
    return { matches, media: query, addEventListener: vi.fn(), removeEventListener: vi.fn(), addListener: vi.fn(), removeListener: vi.fn(), dispatchEvent: vi.fn(), onchange: null }
  }) as typeof window.matchMedia
}

const POI: Poi = {
  id: 'p1', kind: 'point', frame: 'map', x: 1, y: 2, polygon: [], radius_m: 0.5, name: 'Door', note: '',
  status: 'open', created_by: 'user', created_at: 1, updated_at: 1,
}

function renderWithTheme(ui: React.ReactElement) {
  return render(<ThemeProvider theme={theme}>{ui}</ThemeProvider>)
}

beforeEach(() => installMatchMedia())
afterEach(() => cleanup())

describe.each([
  [400, 'phone'],
  [1400, 'desktop'],
])('POI panels at %i px (%s)', (width, cls) => {
  beforeEach(() => {
    viewportWidth = width
  })

  it('list renders inside OverlayPanel when open', () => {
    renderWithTheme(<PoiListPanel pois={[POI]} selectedId={null} open onToggle={vi.fn()} onFocus={vi.fn()} />)
    const panel = screen.getByTestId('overlay-panel')
    expect(panel.getAttribute('data-layout')).toBe(cls)
    expect(panel.contains(screen.getByText('Door'))).toBe(true)
    expect(screen.queryByTestId('overlay-panel-handle') !== null).toBe(cls === 'phone')
  })

  it('list shows only a toggle when closed, and toggling calls onToggle', () => {
    const onToggle = vi.fn()
    renderWithTheme(<PoiListPanel pois={[POI]} selectedId={null} open={false} onToggle={onToggle} onFocus={vi.fn()} />)
    expect(screen.queryByTestId('overlay-panel')).toBeNull()
    fireEvent.click(screen.getByRole('button', { name: 'Expand POI list' }))
    expect(onToggle).toHaveBeenCalledTimes(1)
  })

  it('editor renders inside OverlayPanel and closes', () => {
    const onClose = vi.fn()
    renderWithTheme(<PoiEditorPanel poi={POI} busy={false} onClose={onClose} onSend={vi.fn()} />)
    const panel = screen.getByTestId('overlay-panel')
    expect(panel.getAttribute('data-layout')).toBe(cls)
    expect(panel.contains(screen.getByTestId('poi-editor'))).toBe(true)
    fireEvent.click(screen.getByRole('button', { name: 'Close' }))
    expect(onClose).toHaveBeenCalledTimes(1)
  })

  it('panels never start a map gesture', () => {
    const parent = vi.fn()
    renderWithTheme(
      <div onPointerDown={parent}>
        <PoiListPanel pois={[POI]} selectedId={null} open onToggle={vi.fn()} onFocus={vi.fn()} />
      </div>,
    )
    fireEvent.pointerDown(screen.getByText('Door'))
    fireEvent.pointerDown(screen.getByRole('button', { name: 'Collapse POI list' }))
    expect(parent).not.toHaveBeenCalled()
  })
})
