import { describe, expect, it } from 'vitest'
import { GRASP_MENU, IDLE_PICK, pickStep, strategyForMenuItem } from './pick'

describe('grasp dropdown mapping', () => {
  it('lists Auto, Scoop, Angled, Top down in order', () => {
    expect(GRASP_MENU.map((m) => m.label)).toEqual(['Auto', 'Scoop', 'Angled', 'Top down'])
  })

  it('maps each item to its strategy', () => {
    expect(strategyForMenuItem('Auto')).toBe('auto')
    expect(strategyForMenuItem('Scoop')).toBe('scoop')
    expect(strategyForMenuItem('Angled')).toBe('angled')
    expect(strategyForMenuItem('Top down')).toBe('top_down')
    expect(strategyForMenuItem('Nope')).toBeNull()
  })
})

describe('pick mode state machine', () => {
  it('selecting an item enters pick mode with that strategy and plans nothing', () => {
    const step = pickStep(IDLE_PICK, { type: 'select', strategy: 'scoop' })
    expect(step.state).toEqual({ kind: 'picking', strategy: 'scoop' })
    expect(step.plan).toBeNull()
  })

  it('the next click sets the object, requests a plan and leaves pick mode', () => {
    const picking = pickStep(IDLE_PICK, { type: 'select', strategy: 'angled' }).state
    const step = pickStep(picking, { type: 'click', point: { x: 0.4, y: -0.1 } })
    expect(step.state).toEqual(IDLE_PICK)
    expect(step.plan).toEqual({ strategy: 'angled', point: { x: 0.4, y: -0.1 } })
  })

  it('a click while idle does nothing', () => {
    const step = pickStep(IDLE_PICK, { type: 'click', point: { x: 1, y: 1 } })
    expect(step).toEqual({ state: IDLE_PICK, plan: null })
  })

  it('cancel leaves pick mode without a plan', () => {
    const picking = pickStep(IDLE_PICK, { type: 'select', strategy: 'auto' }).state
    expect(pickStep(picking, { type: 'cancel' })).toEqual({ state: IDLE_PICK, plan: null })
  })

  it('selecting again while picking switches the strategy; a second click after a plan does nothing', () => {
    let s = pickStep(IDLE_PICK, { type: 'select', strategy: 'auto' }).state
    s = pickStep(s, { type: 'select', strategy: 'top_down' }).state
    expect(s).toEqual({ kind: 'picking', strategy: 'top_down' })
    const done = pickStep(s, { type: 'click', point: { x: 0.3, y: 0 } })
    expect(done.plan?.strategy).toBe('top_down')
    expect(pickStep(done.state, { type: 'click', point: { x: 0.5, y: 0 } }).plan).toBeNull()
  })

  it('re-picking after a plan starts a new pick', () => {
    const first = pickStep(pickStep(IDLE_PICK, { type: 'select', strategy: 'auto' }).state, { type: 'click', point: { x: 0.3, y: 0 } })
    const again = pickStep(first.state, { type: 'select', strategy: 'scoop' })
    expect(again.state).toEqual({ kind: 'picking', strategy: 'scoop' })
  })
})
