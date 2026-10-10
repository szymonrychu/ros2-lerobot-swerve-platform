import { describe, expect, it } from 'vitest'
import { extractField } from './fieldExtract'

const JOINTS = { name: ['fl_drive', 'fl_steer', 'fr_steer'], position: [5, 0.3, -0.2] }

describe('extractField', () => {
  it('reads nested fields and indices', () => {
    expect(extractField({ a: { b: 2 } }, 'a.b')).toBe(2)
    expect(extractField(JOINTS, 'position[1]')).toBe(0.3)
  })

  it('selects an array element by the joint name at the same index', () => {
    expect(extractField(JOINTS, 'position[name=fl_steer]')).toBe(0.3)
    expect(extractField(JOINTS, 'position[name=fr_steer]')).toBe(-0.2)
  })

  it('returns null for an unknown name, a missing name array or a short array', () => {
    expect(extractField(JOINTS, 'position[name=rr_steer]')).toBeNull()
    expect(extractField({ position: [1] }, 'position[name=fl_steer]')).toBeNull()
    expect(extractField({ name: ['a', 'b'], position: [1] }, 'position[name=b]')).toBeNull()
  })

  it('returns null for missing paths and non-numbers', () => {
    expect(extractField(null, 'a')).toBeNull()
    expect(extractField(JOINTS, 'name[0]')).toBeNull()
  })
})
