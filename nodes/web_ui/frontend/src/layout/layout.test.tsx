// @vitest-environment jsdom
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { cleanup, fireEvent, render, screen } from '@testing-library/react'
import { ThemeProvider } from '@mui/material/styles'
import { theme, PANEL_BG, TOKENS, TOUCH_TARGET_PX } from '../theme'
import { useLayout } from './useLayout'
import { MapSidebar } from './MapSidebar'
import { OverlayPanel } from './OverlayPanel'
import { designViolations, SOURCES } from './designRules'

let viewportWidth = 1400
let coarsePointer = false

/** Minimal matchMedia: evaluates min-width / max-width px conditions against viewportWidth. */
function installMatchMedia(): void {
  window.matchMedia = ((query: string) => {
    const evaluate = (): boolean => {
      const min = /min-width:\s*([\d.]+)px/.exec(query)
      const max = /max-width:\s*([\d.]+)px/.exec(query)
      if (min && viewportWidth < Number(min[1])) return false
      if (max && viewportWidth > Number(max[1])) return false
      if (/pointer:\s*coarse/.test(query)) return coarsePointer
      return true
    }
    return {
      matches: evaluate(),
      media: query,
      addEventListener: vi.fn(),
      removeEventListener: vi.fn(),
      addListener: vi.fn(),
      removeListener: vi.fn(),
      dispatchEvent: vi.fn(),
      onchange: null,
    }
  }) as typeof window.matchMedia
}

function Probe() {
  return <div data-testid="cls">{useLayout()}</div>
}

function renderWithTheme(ui: React.ReactElement) {
  return render(<ThemeProvider theme={theme}>{ui}</ThemeProvider>)
}

beforeEach(() => {
  coarsePointer = false
  installMatchMedia()
})
afterEach(() => cleanup())

describe('useLayout', () => {
  it.each([
    [320, 'phone'],
    [599, 'phone'],
    [600, 'tablet'],
    [900, 'tablet'],
    [1199, 'tablet'],
    [1200, 'desktop'],
    [1920, 'desktop'],
  ])('width %i is %s', (width, expected) => {
    viewportWidth = width
    renderWithTheme(<Probe />)
    expect(screen.getByTestId('cls').textContent).toBe(expected)
  })
})

describe('OverlayPanel', () => {
  it('renders nothing when closed', () => {
    viewportWidth = 1400
    renderWithTheme(<OverlayPanel title="T" open={false} anchor="left">body</OverlayPanel>)
    expect(screen.queryByTestId('overlay-panel')).toBeNull()
  })

  it('phone: bottom sheet with a square top, no handle, 60dvh max height', () => {
    viewportWidth = 400
    renderWithTheme(<OverlayPanel title="T" open anchor="left">body</OverlayPanel>)
    const panel = screen.getByTestId('overlay-panel')
    expect(panel.getAttribute('data-layout')).toBe('phone')
    expect(panel.style.maxHeight).toBe('60dvh')
    expect(panel.style.bottom).toBe('0px')
    expect(panel.style.borderRadius).toBe('0')
    expect(screen.queryByTestId('overlay-panel-handle')).toBeNull()
  })

  it.each([
    [800, 'tablet'],
    [1400, 'desktop'],
  ])('%i px (%s): floating card, clamp width, no handle', (width, cls) => {
    viewportWidth = width
    renderWithTheme(<OverlayPanel title="T" open anchor="right">body</OverlayPanel>)
    const panel = screen.getByTestId('overlay-panel')
    expect(panel.getAttribute('data-layout')).toBe(cls)
    expect(panel.getAttribute('style')).toContain('clamp(280px, 30vw, 380px)')
    expect(panel.style.right).not.toBe('')
    expect(panel.style.left).toBe('')
    expect(screen.queryByTestId('overlay-panel-handle')).toBeNull()
  })

  it('anchors left, and bottom on wide layouts', () => {
    viewportWidth = 1400
    const { unmount } = renderWithTheme(<OverlayPanel title="T" open anchor="left">b</OverlayPanel>)
    expect(screen.getByTestId('overlay-panel').style.left).not.toBe('')
    unmount()
    renderWithTheme(<OverlayPanel title="T" open anchor="bottom">b</OverlayPanel>)
    const p = screen.getByTestId('overlay-panel')
    expect(p.style.bottom).not.toBe('')
    expect(p.style.top).toBe('')
  })

  it('shows title and children, close button calls onClose', () => {
    viewportWidth = 1400
    const onClose = vi.fn()
    renderWithTheme(<OverlayPanel title="My title" open anchor="left" onClose={onClose}>hello</OverlayPanel>)
    expect(screen.getByText('My title')).toBeTruthy()
    expect(screen.getByText('hello')).toBeTruthy()
    fireEvent.click(screen.getByRole('button', { name: 'Close' }))
    expect(onClose).toHaveBeenCalledTimes(1)
  })

  it('no close button without onClose', () => {
    renderWithTheme(<OverlayPanel title="T" open anchor="left">b</OverlayPanel>)
    expect(screen.queryByRole('button', { name: 'Close' })).toBeNull()
  })

  it('stops pointerdown propagation', () => {
    const parent = vi.fn()
    renderWithTheme(
      <div onPointerDown={parent}>
        <OverlayPanel title="T" open anchor="left">
          <span data-testid="inside">x</span>
        </OverlayPanel>
      </div>,
    )
    fireEvent.pointerDown(screen.getByTestId('inside'))
    expect(parent).not.toHaveBeenCalled()
  })

  it('uses the PANEL_BG token as background', () => {
    expect(PANEL_BG).toBe(TOKENS.bgPanel)
    renderWithTheme(<OverlayPanel title="T" open anchor="left">b</OverlayPanel>)
    expect(screen.getByTestId('overlay-panel').style.background).toBe('rgb(22, 24, 29)') // PANEL_BG, normalised by jsdom
  })

  it.each([400, 1400])('coarse pointer: close is at least TOUCH_TARGET_PX at %i px', (width) => {
    viewportWidth = width
    coarsePointer = true
    renderWithTheme(<OverlayPanel title="T" open anchor="left" onClose={() => undefined}>b</OverlayPanel>)
    const close = screen.getByRole('button', { name: 'Close' })
    expect(parseFloat(close.style.minWidth)).toBeGreaterThanOrEqual(TOUCH_TARGET_PX)
    expect(parseFloat(close.style.minHeight)).toBeGreaterThanOrEqual(TOUCH_TARGET_PX)
  })

  it.each([400, 1400])('fine pointer: close does not stretch the header past panelHeaderPx at %i px', (width) => {
    viewportWidth = width
    renderWithTheme(<OverlayPanel title="T" open anchor="left" onClose={() => undefined}>b</OverlayPanel>)
    const close = screen.getByRole('button', { name: 'Close' })
    expect(parseFloat(close.style.minHeight || '0')).toBeLessThanOrEqual(TOKENS.panelHeaderPx)
    expect(parseFloat(close.style.minWidth || '0')).toBeLessThanOrEqual(TOKENS.panelHeaderPx)
  })

  it('header is panelHeaderPx high on a fine pointer, TOUCH_TARGET_PX on a coarse one', () => {
    const { unmount } = renderWithTheme(<OverlayPanel title="T" open anchor="left">b</OverlayPanel>)
    expect(screen.getByTestId('overlay-panel-header').style.minHeight).toBe(`${TOKENS.panelHeaderPx}px`)
    unmount()
    coarsePointer = true
    renderWithTheme(<OverlayPanel title="T" open anchor="left">b</OverlayPanel>)
    expect(screen.getByTestId('overlay-panel-header').style.minHeight).toBe(`${TOUCH_TARGET_PX}px`)
  })

  it('renders the actions slot in the header', () => {
    renderWithTheme(<OverlayPanel title="T" open anchor="left" actions={<button>Act</button>}>b</OverlayPanel>)
    expect(screen.getByTestId('overlay-panel-header').contains(screen.getByRole('button', { name: 'Act' }))).toBe(true)
  })

  it('is flat: no shadow, no backdrop filter, opaque background', () => {
    renderWithTheme(<OverlayPanel title="T" open anchor="left">b</OverlayPanel>)
    const panel = screen.getByTestId('overlay-panel')
    expect(panel.style.backdropFilter ?? '').toBe('')
    expect(panel.style.boxShadow).toBe('')
  })

  it.each([800, 1400])('inside a MapSidebar at %i px: an in-flow section with the header and a divider below', (width) => {
    viewportWidth = width
    renderWithTheme(
      <MapSidebar>
        <OverlayPanel title="Layers" open anchor="right" onClose={() => undefined}>b</OverlayPanel>
      </MapSidebar>,
    )
    const panel = screen.getByTestId('overlay-panel')
    expect(panel.getAttribute('data-docked')).toBe('true')
    expect(panel.style.position).toBe('relative')
    expect(panel.style.width).toBe('100%')
    expect(panel.getAttribute('style')).not.toContain('clamp(')
    expect(panel.contains(screen.getByTestId('overlay-panel-header'))).toBe(true)
    expect(screen.getByRole('button', { name: 'Close' })).toBeTruthy()
  })

  it('docked prop outside a sidebar: an in-flow card on wide layouts, a bottom sheet on phone', () => {
    viewportWidth = 1400
    const { unmount } = renderWithTheme(<OverlayPanel title="T" open anchor="bottom" docked>b</OverlayPanel>)
    let panel = screen.getByTestId('overlay-panel')
    expect(panel.style.position).toBe('relative')
    expect(panel.getAttribute('data-docked')).toBe('false')
    unmount()
    viewportWidth = 400
    renderWithTheme(<OverlayPanel title="T" open anchor="bottom" docked>b</OverlayPanel>)
    panel = screen.getByTestId('overlay-panel')
    expect(panel.style.position).toBe('absolute')
    expect(panel.style.bottom).toBe('0px')
  })

  it('passes the design rules', () => {
    for (const file of ['layout/OverlayPanel.tsx', 'layout/ConfirmDialog.tsx', 'layout/MapSidebar.tsx']) {
      expect(designViolations(file, SOURCES[file])).toEqual([])
    }
  })
})
