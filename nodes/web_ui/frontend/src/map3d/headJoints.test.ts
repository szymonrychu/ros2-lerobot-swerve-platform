import { describe, expect, it } from 'vitest'
import { headJointValues } from './headJoints'

describe('headJointValues', () => {
  it('maps a message to head_pan / head_tilt values by name', () => {
    const out = headJointValues({ name: ['head_tilt', 'head_pan'], position: [0.2, -0.4] })
    expect(out).toEqual({ name: ['head_pan', 'head_tilt'], position: [-0.4, 0.2] })
  })

  it('is undefined before a message', () => {
    expect(headJointValues(undefined)).toBeUndefined()
  })

  it('is undefined when either joint is missing', () => {
    expect(headJointValues({ name: ['head_pan'], position: [0.1] })).toBeUndefined()
    expect(headJointValues({ name: ['head_pan', 'head_tilt'], position: [0.1] })).toBeUndefined()
    expect(headJointValues({})).toBeUndefined()
  })

  it('is undefined when either value is non-finite', () => {
    expect(headJointValues({ name: ['head_pan', 'head_tilt'], position: [NaN, 0] })).toBeUndefined()
    expect(headJointValues({ name: ['head_pan', 'head_tilt'], position: [0, Infinity] })).toBeUndefined()
  })
})
