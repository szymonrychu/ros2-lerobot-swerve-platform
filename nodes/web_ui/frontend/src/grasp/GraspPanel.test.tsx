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
    setFrame: noop,
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
    setPickMode: noop,
    applyPick: noop,
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

const render = (s: GraspPanelState) => renderToStaticMarkup(<GraspPanel state={s} topView onClose={() => undefined} />)

describe('GraspPanel', () => {
  it('shows the inputs and the action buttons, Execute disabled without a plan', () => {
    const html = render(state())
    for (const text of ['Plan', 'Execute', 'Release', 'Stop', 'Strategy', 'Advanced', 'Pick on map']) {
      expect(html).toContain(text)
    }
    expect(html).toMatch(/<button[^>]*disabled[^>]*>Execute/)
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
