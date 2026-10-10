import { describe, expect, it } from 'vitest'
import { advanceBaseJoints, MAX_INTEGRATION_DT_S, wrapAngle } from './baseJoints'

const MSG = {
  name: ['fl_drive', 'fl_steer', 'fr_drive', 'fr_steer'],
  position: [5.9, 0.3, 1.1, -0.2],
  velocity: [2, 0, -1, 0],
}

function value(js: { name: string[]; position: number[] }, joint: string): number {
  return js.position[js.name.indexOf(joint)]
}

describe('wrapAngle', () => {
  it('wraps into [-pi, pi)', () => {
    expect(wrapAngle(0.5)).toBeCloseTo(0.5)
    expect(wrapAngle(Math.PI + 0.1)).toBeCloseTo(-Math.PI + 0.1)
    expect(wrapAngle(-Math.PI - 0.1)).toBeCloseTo(Math.PI - 0.1)
    expect(wrapAngle(4 * Math.PI + 0.2)).toBeCloseTo(0.2)
  })
})

describe('advanceBaseJoints', () => {
  it('passes steering positions through unchanged', () => {
    const out = advanceBaseJoints(undefined, MSG, 0.1)
    expect(value(out, 'fl_steer')).toBe(0.3)
    expect(value(out, 'fr_steer')).toBe(-0.2)
  })

  it('starts the drive angle at zero (the encoder position wraps and is not used)', () => {
    const out = advanceBaseJoints(undefined, MSG, 0.1)
    expect(value(out, 'fl_drive')).toBeCloseTo(0.2)
    expect(value(out, 'fr_drive')).toBeCloseTo(-0.1)
  })

  it('integrates the wheel velocity over dt, positive = forward', () => {
    const first = advanceBaseJoints(undefined, MSG, 0.1)
    const second = advanceBaseJoints(first, MSG, 0.1)
    expect(value(second, 'fl_drive')).toBeCloseTo(0.4)
    expect(value(second, 'fr_drive')).toBeCloseTo(-0.2)
  })

  it('wraps the accumulated angle', () => {
    let state = advanceBaseJoints(undefined, { name: ['fl_drive'], velocity: [3] }, 0)
    for (let i = 0; i < 100; i++) state = advanceBaseJoints(state, { name: ['fl_drive'], velocity: [3] }, 0.1)
    expect(Math.abs(value(state, 'fl_drive'))).toBeLessThanOrEqual(Math.PI)
    // 100 * 0.3 = 30 rad -> wrapped
    expect(value(state, 'fl_drive')).toBeCloseTo(wrapAngle(30))
  })

  it('caps dt so a stalled stream does not spin the wheel', () => {
    const out = advanceBaseJoints(undefined, { name: ['fl_drive'], velocity: [1] }, 60)
    expect(value(out, 'fl_drive')).toBeCloseTo(MAX_INTEGRATION_DT_S)
  })

  it('ignores negative dt and non-finite velocities', () => {
    const start = advanceBaseJoints(undefined, MSG, 0.1)
    const back = advanceBaseJoints(start, MSG, -1)
    expect(value(back, 'fl_drive')).toBeCloseTo(value(start, 'fl_drive'))
    const nan = advanceBaseJoints(start, { name: ['fl_drive'], velocity: [NaN] }, 0.1)
    expect(value(nan, 'fl_drive')).toBeCloseTo(value(start, 'fl_drive'))
    const nul = advanceBaseJoints(start, { name: ['fl_drive'], velocity: [null as unknown as number] }, 0.1)
    expect(value(nul, 'fl_drive')).toBeCloseTo(value(start, 'fl_drive'))
  })

  it('keeps drive angles of joints missing from the message and skips non-finite steering', () => {
    const start = advanceBaseJoints(undefined, MSG, 0.1)
    const out = advanceBaseJoints(start, { name: ['fl_steer', 'fr_steer'], position: [NaN, 0.5] }, 0.1)
    expect(value(out, 'fl_drive')).toBeCloseTo(value(start, 'fl_drive'))
    expect(out.name).not.toContain('fl_steer')
    expect(value(out, 'fr_steer')).toBe(0.5)
  })

  it('handles an empty or malformed message', () => {
    expect(advanceBaseJoints(undefined, {}, 0.1)).toEqual({ name: [], position: [] })
    const out = advanceBaseJoints(undefined, { name: ['fl_drive'] }, 0.1)
    expect(out.name).toEqual([])
  })
})
