import { describe, expect, it } from 'vitest'
import { robotIconGeometry } from './robotIconGeometry'

const MSG = {
  name: ['fr_steer', 'fl_drive', 'rl_steer', 'fl_steer', 'rr_steer'],
  position: [-0.5, 9, 0.25, 0.4, 0],
}

function roller(g: ReturnType<typeof robotIconGeometry>, id: string) {
  const r = g.rollers.find((x) => x.id === id)
  if (!r) throw new Error(id)
  return r
}

describe('robotIconGeometry', () => {
  it('draws the footprint to scale with the front up', () => {
    const g = robotIconGeometry(undefined)
    expect(g.footprint.height / g.footprint.width).toBeCloseTo(0.47 / 0.386)
    expect(g.footprint.x).toBeCloseTo(-g.footprint.width / 2)
    expect(g.footprint.y).toBeCloseTo(-g.footprint.height / 2)
  })

  it('places the four rollers at the real module positions (fl up-left, rr down-right)', () => {
    const g = robotIconGeometry(undefined)
    const scale = g.footprint.height / 0.47
    expect(g.rollers.map((r) => r.id)).toEqual(['fl', 'fr', 'rl', 'rr'])
    expect(roller(g, 'fl').cx).toBeCloseTo(-0.1333 * scale)
    expect(roller(g, 'fl').cy).toBeCloseTo(-0.1525 * scale)
    expect(roller(g, 'rr').cx).toBeCloseTo(0.1333 * scale)
    expect(roller(g, 'rr').cy).toBeCloseTo(0.1525 * scale)
  })

  it('sizes each roller as diameter x width of the real wheel', () => {
    const g = robotIconGeometry(undefined)
    const scale = g.footprint.height / 0.47
    expect(roller(g, 'fl').length).toBeCloseTo(0.12 * scale)
    expect(roller(g, 'fl').width).toBeCloseTo(0.03 * scale)
  })

  it('is straight and inactive before the first message', () => {
    const g = robotIconGeometry(undefined)
    expect(g.active).toBe(false)
    expect(g.rollers.every((r) => r.rotationDeg === 0)).toBe(true)
  })

  it('rotates each roller by its steering angle matched by name, CCW positive seen from above', () => {
    const g = robotIconGeometry(MSG)
    expect(g.active).toBe(true)
    expect(roller(g, 'fl').rotationDeg).toBeCloseTo(-(0.4 * 180) / Math.PI)
    expect(roller(g, 'fr').rotationDeg).toBeCloseTo((0.5 * 180) / Math.PI)
    expect(roller(g, 'rl').rotationDeg).toBeCloseTo(-(0.25 * 180) / Math.PI)
    expect(roller(g, 'rr').rotationDeg).toBeCloseTo(0)
  })

  it('keeps a roller straight when its joint is missing or not finite', () => {
    const g = robotIconGeometry({ name: ['fl_steer', 'fr_steer'], position: [0.4, Number.NaN] })
    expect(roller(g, 'fr').rotationDeg).toBe(0)
    expect(roller(g, 'rl').rotationDeg).toBe(0)
    expect(roller(g, 'fl').rotationDeg).not.toBe(0)
  })

  it('treats a message without steering joints as inactive', () => {
    expect(robotIconGeometry({ name: ['fl_drive'], position: [1] }).active).toBe(false)
  })

  it('puts a front marker on the top edge of the footprint', () => {
    const g = robotIconGeometry(undefined)
    expect(g.frontMarker.tipY).toBeLessThan(g.footprint.y)
    expect(g.frontMarker.tipX).toBe(0)
  })
})
