// @vitest-environment jsdom
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { cleanup, fireEvent, render, screen } from '@testing-library/react'
import { ThemeProvider } from '@mui/material/styles'
import { theme, PANEL_BG, TOKENS, TOUCH_TARGET_PX } from '../theme'
import { useLayout } from './useLayout'
import { OverlayPanel } from './OverlayPanel'

let viewportWidth = 1400

/** Minimal matchMedia: evaluates min-width / max-width px conditions against viewportWidth. */
function installMatchMedia(): void {
  window.matchMedia = ((query: string) => {
    const evaluate = (): boolean => {
      const min = /min-width:\s*([\d.]+)px/.exec(query)
      const max = /max-width:\s*([\d.]+)px/.exec(query)
      if (min && viewportWidth < Number(min[1])) return false
      if (max && viewportWidth > Number(max[1])) return false
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

beforeEach(() => installMatchMedia())
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

  it('phone: bottom sheet with drag handle and 60dvh max height', () => {
    viewportWidth = 400
    renderWithTheme(<OverlayPanel title="T" open anchor="left">body</OverlayPanel>)
    const panel = screen.getByTestId('overlay-panel')
    expect(panel.getAttribute('data-layout')).toBe('phone')
    expect(panel.style.maxHeight).toBe('60dvh')
    expect(panel.style.bottom).toBe('0px')
    expect(screen.getByTestId('overlay-panel-handle')).toBeTruthy()
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

  it.each([400, 1400])('touch targets are at least TOUCH_TARGET_PX at %i px', (width) => {
    viewportWidth = width
    renderWithTheme(<OverlayPanel title="T" open anchor="left" onClose={() => undefined}>b</OverlayPanel>)
    const close = screen.getByRole('button', { name: 'Close' })
    expect(parseFloat(close.style.minWidth)).toBeGreaterThanOrEqual(TOUCH_TARGET_PX)
    expect(parseFloat(close.style.minHeight)).toBeGreaterThanOrEqual(TOUCH_TARGET_PX)
    if (width < 600) {
      const handle = screen.getByTestId('overlay-panel-handle')
      expect(parseFloat(handle.style.minHeight)).toBeGreaterThanOrEqual(TOUCH_TARGET_PX)
    }
  })
})
