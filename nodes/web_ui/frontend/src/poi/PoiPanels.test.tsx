// @vitest-environment jsdom
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { cleanup, fireEvent, render, screen, within } from '@testing-library/react'
import { ThemeProvider } from '@mui/material/styles'
import { theme } from '../theme'
import { PoiEditorPanel, PoiListPanel } from './PoiPanels'
import { designViolations, SOURCES } from '../layout/designRules'
import { POI_NAME_MAX } from './types'
import type { Poi } from './types'
import type { Photo } from '../photos/api'

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
    expect(screen.queryByTestId('overlay-panel-handle')).toBeNull()
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

describe('POI panels declutter', () => {
  beforeEach(() => {
    viewportWidth = 1400
  })

  it('shows the status as text, with a state mark only for open', () => {
    const done: Poi = { ...POI, id: 'p2', name: 'Shelf', status: 'done' }
    const { container } = renderWithTheme(<PoiListPanel pois={[POI, done]} selectedId={null} open onToggle={vi.fn()} onFocus={vi.fn()} />)
    expect(container.querySelector('.MuiChip-root')).toBeNull()
    expect(screen.getByText('open')).toBeTruthy()
    expect(screen.getByText('done')).toBeTruthy()
    expect(screen.getAllByTestId('status-mark')).toHaveLength(1)
  })

  it('draws the open state mark as a 6 px square in a neutral colour, not the warning hue', () => {
    renderWithTheme(<PoiListPanel pois={[POI]} selectedId={null} open onToggle={vi.fn()} onFocus={vi.fn()} />)
    const style = getComputedStyle(screen.getByTestId('status-mark'))
    expect(style.width).toBe('6px')
    expect(style.height).toBe('6px')
    expect(style.borderRadius).not.toBe('50%')
    expect(style.backgroundColor).not.toBe('')
    expect(style.backgroundColor).not.toBe(getComputedStyle(document.createElement('div')).backgroundColor)
    const warning = document.createElement('div')
    warning.style.backgroundColor = theme.palette.warning.main
    expect(style.backgroundColor).not.toBe(warning.style.backgroundColor)
  })

  it('has no Collapse text and shows the count once', () => {
    renderWithTheme(<PoiListPanel pois={[POI]} selectedId={null} open onToggle={vi.fn()} onFocus={vi.fn()} />)
    expect(screen.queryByText('Collapse')).toBeNull()
    expect(screen.getAllByText(/Points of interest \(1\)/)).toHaveLength(1)
    fireEvent.click(screen.getByRole('button', { name: 'Collapse POI list' }))
  })

  it('collapse button in the header calls onToggle', () => {
    const onToggle = vi.fn()
    renderWithTheme(<PoiListPanel pois={[POI]} selectedId={null} open onToggle={onToggle} onFocus={vi.fn()} />)
    fireEvent.click(screen.getByRole('button', { name: 'Collapse POI list' }))
    expect(onToggle).toHaveBeenCalledTimes(1)
  })

  it('editor has no drag hint and shows the counter only within 10% of the limit', () => {
    renderWithTheme(<PoiEditorPanel poi={POI} busy={false} onClose={vi.fn()} onSend={vi.fn()} />)
    expect(screen.queryByText(/Drag the selected/)).toBeNull()
    expect(screen.queryByText(new RegExp(`/${POI_NAME_MAX}$`))).toBeNull()
    fireEvent.change(screen.getByLabelText('Name'), { target: { value: 'x'.repeat(Math.ceil(POI_NAME_MAX * 0.9)) } })
    expect(screen.getByText(`${Math.ceil(POI_NAME_MAX * 0.9)}/${POI_NAME_MAX}`)).toBeTruthy()
  })

  it('delete confirms in the shared ConfirmDialog and sends only after confirming', () => {
    const onSend = vi.fn(async () => ({ ok: true, message: 'ok' }))
    renderWithTheme(<PoiEditorPanel poi={POI} busy={false} onClose={vi.fn()} onSend={onSend as never} />)
    fireEvent.click(screen.getByRole('button', { name: 'Delete' }))
    expect(onSend).not.toHaveBeenCalled()
    expect(screen.getByRole('dialog')).toBeTruthy()
    fireEvent.click(within(screen.getByRole('dialog')).getByRole('button', { name: 'Delete' }))
    expect(onSend).toHaveBeenCalledWith({ op: 'delete', poi: { id: 'p1' } }, 'POI deleted')
  })

  it('uses the default button size', () => {
    renderWithTheme(<PoiEditorPanel poi={POI} busy={false} onClose={vi.fn()} onSend={vi.fn()} />)
    expect(document.querySelector('.MuiButton-sizeSmall')).toBeNull()
  })

  it('collapse icon button is the default size', () => {
    renderWithTheme(<PoiListPanel pois={[POI]} selectedId={null} open onToggle={vi.fn()} onFocus={vi.fn()} />)
    expect(document.querySelector('.MuiIconButton-sizeSmall')).toBeNull()
  })

  it('passes the design rules', () => {
    expect(designViolations('poi/PoiPanels.tsx', SOURCES['poi/PoiPanels.tsx'])).toEqual([])
  })
})

describe('Clear POIs button', () => {
  it('opens the clear dialog only when a tab id is given', () => {
    viewportWidth = 1400
    const { unmount } = renderWithTheme(<PoiListPanel pois={[POI]} selectedId={null} open onToggle={vi.fn()} onFocus={vi.fn()} />)
    expect(screen.queryByRole('button', { name: 'Clear POIs' })).toBeNull()
    unmount()
    renderWithTheme(<PoiListPanel pois={[POI]} selectedId={null} open onToggle={vi.fn()} onFocus={vi.fn()} tabId="map" />)
    fireEvent.click(screen.getByRole('button', { name: 'Clear POIs' }))
    expect(screen.getByTestId('clear-pois-preview').textContent).toContain('1')
  })
})

describe('Clear POIs dialog photos', () => {
  it('counts the photos handed to the list panel in the clear preview', () => {
    viewportWidth = 1400
    const photo = { id: 'f1', createdBy: 'agent', x: 1, y: 2 } as unknown as Photo
    renderWithTheme(<PoiListPanel pois={[POI]} photos={[photo, { ...photo, id: 'f2' }]} selectedId={null} open onToggle={vi.fn()} onFocus={vi.fn()} tabId="map" />)
    fireEvent.click(screen.getByRole('button', { name: 'Clear POIs' }))
    expect(screen.getByTestId('clear-pois-preview').textContent).toBe('Removes 1 POIs and 2 photos')
  })
})

describe('POI categories', () => {
  const tagged = (note: string): Poi => ({ ...POI, note })

  it('list row shows the category chip, problem type and survey marker', () => {
    renderWithTheme(<PoiListPanel pois={[tagged('#problem/window #survey-spot\nglass')]} selectedId={null} open onToggle={vi.fn()} onFocus={vi.fn()} />)
    expect(screen.getByTestId('poi-category-chip').textContent).toBe('Problem')
    expect(screen.getByTestId('poi-problem-type').textContent).toBe('window')
    expect(screen.getByTestId('poi-survey-marker')).toBeTruthy()
  })
  it('list row without tags has no chip', () => {
    renderWithTheme(<PoiListPanel pois={[POI]} selectedId={null} open onToggle={vi.fn()} onFocus={vi.fn()} />)
    expect(screen.queryByTestId('poi-category-chip')).toBeNull()
    expect(screen.queryByTestId('poi-survey-marker')).toBeNull()
  })
  it('editor shows chip and survey marker, text view without the tag line, field keeps the full note', () => {
    renderWithTheme(<PoiEditorPanel poi={tagged('#keepout #survey\nstairs below')} busy={false} onClose={vi.fn()} onSend={vi.fn()} />)
    expect(screen.getByTestId('poi-category-chip').textContent).toBe('Keep-out')
    expect(screen.getByTestId('poi-survey-marker')).toBeTruthy()
    expect(screen.getByTestId('poi-note-text').textContent).toBe('stairs below')
    expect((screen.getByLabelText('Note') as HTMLTextAreaElement).value).toBe('#keepout #survey\nstairs below')
  })
})

describe('marker POI editor', () => {
  const MARKER: Poi = {
    ...POI, id: 'm1', kind: 'marker', name: 'Charger', radius_m: 0, z: 0.45, orientation: [0, 0, 0, 1],
    times_seen: 7, first_seen: 1, last_seen: Date.now() / 1000 - 120, confidence: 0.8,
    marker: { kind: 'charger', id: 12, key: 'charger:12', size_mm: 80, params: {}, payload: 'p', version: 1, pose_samples: 5 },
  }

  it('shows the marker fields and no position or radius editor', () => {
    viewportWidth = 1400
    renderWithTheme(<PoiEditorPanel poi={MARKER} busy={false} onClose={vi.fn()} onSend={vi.fn()} />)
    const details = screen.getByTestId('poi-marker-details').textContent ?? ''
    for (const part of ['charger', '12', '80 mm', '0.45 m', '7', '2 min ago', '80%']) expect(details).toContain(part)
    expect(screen.queryByLabelText(/Radius/)).toBeNull()
    expect(screen.queryByText(/at 1\.00, 2\.00/)).toBeNull()
    expect(screen.getByLabelText('Name')).toBeTruthy()
    expect(screen.getByLabelText('Note')).toBeTruthy()
    expect(screen.getByRole('button', { name: 'Delete' })).toBeTruthy()
  })

  it('editing the name sends only name, note and status', () => {
    viewportWidth = 1400
    const onSend = vi.fn(async () => ({ ok: true, message: 'ok' }))
    renderWithTheme(<PoiEditorPanel poi={MARKER} busy={false} onClose={vi.fn()} onSend={onSend as never} />)
    fireEvent.change(screen.getByLabelText('Name'), { target: { value: 'Dock' } })
    fireEvent.click(screen.getByRole('button', { name: 'Save' }))
    expect(onSend).toHaveBeenCalledWith({ op: 'update', poi: { id: 'm1', name: 'Dock', note: '', status: 'open' } }, 'POI saved')
  })

  it('the list shows a marker row', () => {
    viewportWidth = 1400
    renderWithTheme(<PoiListPanel pois={[MARKER]} selectedId={null} open onToggle={vi.fn()} onFocus={vi.fn()} />)
    expect(screen.getByText(/marker - user/)).toBeTruthy()
  })
})
