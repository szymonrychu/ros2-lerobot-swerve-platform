// @vitest-environment jsdom
import { afterEach, describe, expect, it, vi } from 'vitest'
import { cleanup, render as rtlRender, screen, within } from '@testing-library/react'
import { ThemeProvider } from '@mui/material/styles'
import { theme } from '../theme'
import { renderToStaticMarkup } from 'react-dom/server'
import { GraspPanel } from './GraspPanel'
import { DEFAULT_FORM } from './grasp'
import type { GraspPanelState } from './useGrasp'
import { designViolations, SOURCES } from '../layout/designRules'

function state(patch: Partial<GraspPanelState> = {}): GraspPanelState {
  const noop = vi.fn()
  return {
    form: DEFAULT_FORM,
    setForm: noop,
    setParam: noop,
    plan: null,
    planFresh: false,
    planning: false,
    errors: [],
    showErrors: false,
    canExecute: false,
    confirming: false,
    executing: false,
    stream: null,
    lastAnswer: null,
    releaseArmed: false,
    pickMode: false,
    pickStrategy: null,
    startPick: noop,
    cancelPick: noop,
    applyClick: noop,
    hasTarget: false,
    preview: null,
    requestPlan: noop,
    requestExecute: noop,
    cancelConfirm: noop,
    confirmExecute: noop,
    release: noop,
    stop: noop,
    ...patch,
  }
}

const render = (s: GraspPanelState) => renderToStaticMarkup(<GraspPanel state={s} onClose={() => undefined} />)

describe('GraspPanel', () => {
  it('has no position inputs and no strategy selector', () => {
    const html = render(state())
    for (const text of ['Object', 'Advanced', 'Release']) expect(html).toContain(text)
    for (const text of ['Pick on map', 'Strategy', 'base_link', 'label="x', 'Support z']) expect(html).not.toContain(text)
    expect(html).not.toMatch(/<label[^>]*>x</)
    expect(html).not.toContain('Choose Auto, Scoop, Angled or Top down')
  })

  it('has no pick banner and no panel stop (the map status line and toolbar own them)', () => {
    const html = render(state({ pickMode: true, pickStrategy: 'scoop' }))
    expect(html).not.toContain('Click the object on the map')
    expect(html).not.toContain('Cancel')
    expect(html).not.toMatch(/>\s*Stop\s*</)
    expect(render(state())).not.toMatch(/>\s*Stop\s*</)
  })

  it('offers Plan again once an object was clicked, Execute only for a feasible fresh plan', () => {
    expect(render(state())).not.toContain('Plan again')
    expect(render(state({ hasTarget: true }))).toContain('Plan again')
    expect(render(state({ hasTarget: true }))).not.toContain('>Execute<')
    const feasible = {
      key: 'k',
      outcome: {
        ok: true, accepted: false, requestId: 'r', action: 'plan', outcome: 'planned' as const, reasons: [], steps: [],
        plan: { strategy: 'angled', feasible: true, reasons: [], skim: false, waypoints: [], slowZone: [], attempts: [] },
      },
    }
    const html = render(state({ hasTarget: true, plan: feasible, planFresh: true, canExecute: true }))
    expect(html).toMatch(/<button[^>]*>Execute/)
    expect(render(state({ hasTarget: true, plan: feasible, planFresh: false }))).not.toMatch(/<button[^>]*>Execute/)
  })

  it('shows the gap below field for Scoop only', () => {
    expect(render(state({ form: { ...DEFAULT_FORM, strategy: 'scoop' } }))).toContain('Gap below (m)')
    expect(render(state({ form: { ...DEFAULT_FORM, strategy: 'auto' } }))).not.toContain('Gap below (m)')
  })

  it('has no panel STOP GRASP while executing (the toolbar owns it)', () => {
    const html = render(state({ executing: true, stream: { requestId: 'r', action: 'execute', state: 'running', outcome: null } }))
    expect(html).not.toContain('STOP GRASP')
    expect(html).toContain('Running execute')
    expect(html).not.toContain('toolbar stop')
  })

  it('shows the confirm step with the plan summary', () => {
    const plan = {
      key: 'k',
      outcome: {
        ok: true, accepted: false, requestId: 'r', action: 'plan', outcome: 'planned' as const, reasons: [], steps: [],
        plan: { strategy: 'scoop', feasible: true, reasons: [], skim: false, waypoints: [], slowZone: [], attempts: [] },
      },
    }
    const html = render(state({ plan, planFresh: true, canExecute: true, confirming: true }))
    expect(html).toContain('Confirm execute')
    expect(html).toContain('Strategy: scoop')
  })

  it('shows reasons of an infeasible plan', () => {
    const outcome = {
      ok: true, accepted: false, requestId: 'r', action: 'plan', outcome: 'infeasible' as const, reasons: ['out of reach'], steps: [],
      plan: { strategy: 'scoop', feasible: false, reasons: ['out of reach'], skim: false, waypoints: [], slowZone: [], attempts: [] },
    }
    const html = render(state({ plan: { key: 'k', outcome } }))
    expect(html).toContain('Infeasible')
    expect(html).toContain('out of reach')
  })

  it('shows validation errors after an attempt', () => {
    expect(render(state({ errors: ['x must be a number'], showErrors: true }))).toContain('x must be a number')
    expect(render(state({ errors: ['x must be a number'], showErrors: false }))).not.toContain('x must be a number')
  })

  it('has a grip strength selector showing the current profile', () => {
    const html = render(state({ form: { ...DEFAULT_FORM, gripProfile: 'firm' } }))
    expect(html).toContain('Grip strength')
    for (const label of ['Gentle', 'Soft', 'Normal', 'Firm']) expect(html).toContain(label)
    const pressed = [...html.matchAll(/<button[^>]*aria-pressed="true"[^>]*>([A-Za-z]+)/g)].map((m) => m[1])
    expect(pressed).toEqual(['Firm'])
  })

  it('shows the grip profile and holding load of a grasp result', () => {
    const run = {
      ok: true, accepted: false, requestId: 'r', action: 'execute', outcome: 'grasped' as const, reasons: [], steps: [],
      gripProfile: 'gentle', holdingLoad: 131, slipping: false, crushRisk: false,
    }
    const html = render(state({ stream: { requestId: 'r', action: 'execute', state: 'done', outcome: run } }))
    expect(html).toContain('Grip gentle, holding load 131')
  })
})


let viewportWidth = 1400

function installMatchMedia(): void {
  window.matchMedia = ((query: string) => {
    const min = /min-width:\s*([\d.]+)px/.exec(query)
    const matches = !(min && viewportWidth < Number(min[1]))
    return { matches, media: query, addEventListener: vi.fn(), removeEventListener: vi.fn(), addListener: vi.fn(), removeListener: vi.fn(), dispatchEvent: vi.fn(), onchange: null }
  }) as typeof window.matchMedia
}

describe('GraspPanel on OverlayPanel', () => {
  afterEach(cleanup)

  it('is a bottom sheet without a handle on a phone', () => {
    viewportWidth = 390
    installMatchMedia()
    rtlRender(<ThemeProvider theme={theme}><GraspPanel state={state()} onClose={() => undefined} /></ThemeProvider>)
    const panel = screen.getByTestId('overlay-panel')
    expect(panel.getAttribute('data-layout')).toBe('phone')
    expect(screen.queryByTestId('overlay-panel-handle')).toBeNull()
    expect(panel.style.background).toBe('rgb(22, 24, 29)') // PANEL_BG, normalised by jsdom
  })

  it('hides the form while picking on a phone', () => {
    viewportWidth = 390
    installMatchMedia()
    rtlRender(<ThemeProvider theme={theme}><GraspPanel state={state({ pickMode: true, pickStrategy: 'scoop' })} onClose={() => undefined} /></ThemeProvider>)
    expect(screen.queryByText(/Click the object on the map/)).toBeNull()
    const form = screen.getByText('Object').closest('button')!.parentElement!
    expect(getComputedStyle(form).display).toBe('none')
  })

  it('keeps the form visible while picking on desktop', () => {
    viewportWidth = 1400
    installMatchMedia()
    rtlRender(<ThemeProvider theme={theme}><GraspPanel state={state({ pickMode: true, pickStrategy: 'scoop' })} onClose={() => undefined} /></ThemeProvider>)
    const form = screen.getByText('Object').closest('button')!.parentElement!
    expect(getComputedStyle(form).display).not.toBe('none')
  })

  it('is a floating card without a handle on desktop, and closes', () => {
    viewportWidth = 1400
    installMatchMedia()
    const onClose = vi.fn()
    rtlRender(<ThemeProvider theme={theme}><GraspPanel state={state()} onClose={onClose} /></ThemeProvider>)
    expect(screen.getByTestId('overlay-panel').getAttribute('data-layout')).toBe('desktop')
    expect(screen.queryByTestId('overlay-panel-handle')).toBeNull()
    screen.getByLabelText('Close').click()
    expect(onClose).toHaveBeenCalled()
  })
})

const RUN_OK = {
  ok: true, accepted: false, requestId: 'r', action: 'execute', outcome: 'grasped' as const, reasons: ['note a'], steps: [{ label: 'close', status: 'ok', message: '' }],
  plan: { strategy: 'angled', approachPitchDeg: 45, feasible: true, reasons: [], skim: false, waypoints: [], slowZone: [], attempts: [] },
  gripProfile: 'firm', holdingLoad: 90, slipping: true, crushRisk: false, gripperPositionRad: 0.12, gripperEffort: 30,
}

describe('GraspPanel declutter', () => {
  afterEach(cleanup)
  const mount = (s: GraspPanelState) => {
    viewportWidth = 1400
    installMatchMedia()
    return rtlRender(<ThemeProvider theme={theme}><GraspPanel state={s} onClose={() => undefined} /></ThemeProvider>)
  }

  it('has no button named stop in any state', () => {
    const busy = { requestId: 'r', action: 'execute', state: 'running' as const, outcome: null }
    for (const s of [state(), state({ executing: true, stream: busy }), state({ pickMode: true, pickStrategy: 'scoop' }), state({ hasTarget: true, planning: true })]) {
      mount(s)
      expect(screen.queryAllByRole('button', { name: /stop/i })).toEqual([])
      cleanup()
    }
  })

  it('shows the grasp result as key-value rows, not alerts', () => {
    mount(state({ stream: { requestId: 'r', action: 'execute', state: 'done', outcome: RUN_OK } }))
    expect(screen.queryAllByRole('alert')).toEqual([])
    const result = screen.getByTestId('grasp-result')
    for (const label of ['Result', 'Strategy', 'Reason', 'Step', 'Hold', 'Gripper']) expect(within(result).getByText(label)).toBeTruthy()
    expect(within(result).getByText('angled, pitch 45 deg')).toBeTruthy()
    expect(within(result).getByText('Grip firm, holding load 90, slipping: try a firmer grip')).toBeTruthy()
    expect(within(result).getAllByTestId('status-mark').length).toBeGreaterThan(0)
  })

  it('shows a failed answer as one error row', () => {
    mount(state({ lastAnswer: { ok: false, accepted: false, requestId: null, action: 'plan', error: 'boom', reasons: [], steps: [] } }))
    expect(screen.queryAllByRole('alert')).toEqual([])
    expect(screen.getByText('boom')).toBeTruthy()
  })

  it('shows Planning and Running as one status row', () => {
    const busy = { requestId: 'r', action: 'execute', state: 'running' as const, outcome: null }
    mount(state({ hasTarget: true, planning: true, stream: busy }))
    expect(screen.getAllByText(/Planning|Running/)).toHaveLength(1)
    expect(screen.getAllByText('Status')).toHaveLength(1)
    cleanup()
    mount(state({ stream: busy }))
    expect(screen.getByText('Running execute')).toBeTruthy()
  })

  it('has at most one helper caption and no pick hint', () => {
    const stale = { key: 'k', outcome: RUN_OK }
    const { container } = mount(state({ hasTarget: true, plan: stale, planFresh: false }))
    expect(container.ownerDocument.querySelectorAll('[data-testid="grasp-caption"]').length).toBeLessThanOrEqual(1)
    expect(screen.getByText(/Settings changed since the last plan/)).toBeTruthy()
    expect(screen.queryByText(/Choose Auto|Click the object|Object at/)).toBeNull()
  })

  it('uses the default button size everywhere', () => {
    const stale = { key: 'k', outcome: RUN_OK }
    const { container } = mount(state({ hasTarget: true, plan: stale, planFresh: true, canExecute: true }))
    expect(container.ownerDocument.querySelector('.MuiButton-sizeSmall')).toBeNull()
    expect(container.ownerDocument.querySelector('.MuiButton-sizeLarge')).toBeNull()
  })

  it('has no design violations in the panel and the map tab', () => {
    for (const path of ['grasp/GraspPanel.tsx', 'tabs/MapNavTab.tsx']) {
      expect(SOURCES[path], path).toBeTruthy()
      expect(designViolations(path, SOURCES[path])).toEqual([])
    }
  })
})
