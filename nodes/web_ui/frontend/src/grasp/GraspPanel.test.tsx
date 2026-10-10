import { describe, expect, it, vi } from 'vitest'
import { renderToStaticMarkup } from 'react-dom/server'
import { GraspPanel } from './GraspPanel'
import { DEFAULT_FORM } from './grasp'
import type { GraspPanelState } from './useGrasp'

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
    for (const text of ['Object', 'Advanced', 'Release', 'Stop']) expect(html).toContain(text)
    for (const text of ['Pick on map', 'Strategy', 'base_link', 'label="x', 'Support z']) expect(html).not.toContain(text)
    expect(html).not.toMatch(/<label[^>]*>x</)
    expect(html).toContain('Choose Auto, Scoop, Angled or Top down')
  })

  it('shows the pick banner with Cancel while picking', () => {
    const html = render(state({ pickMode: true, pickStrategy: 'scoop' }))
    expect(html).toContain('Click the object on the map')
    expect(html).toContain('Cancel')
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

  it('keeps a prominent Stop while executing', () => {
    const html = render(state({ executing: true, stream: { requestId: 'r', action: 'execute', state: 'running', outcome: null } }))
    expect(html).toContain('STOP GRASP')
    expect(html).toContain('Running execute')
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
})
