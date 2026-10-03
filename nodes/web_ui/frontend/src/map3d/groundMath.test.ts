import { describe, expect, it } from 'vitest'
import { mapPixelToWorld } from '../map/mapMath'
import { classifyBaseLink, fitDistance, mapBounds, mapPlacement } from './groundMath'

const META = { width: 200, height: 100, resolution: 0.05, origin: { x: -3, y: -1, yaw: 0 } }

describe('mapPlacement', () => {
  it('centres the ground plane on the image centre with metric size', () => {
    const p = mapPlacement(META)
    expect(p.width).toBeCloseTo(10)
    expect(p.height).toBeCloseTo(5)
    expect(p.center.x).toBeCloseTo(2)
    expect(p.center.y).toBeCloseTo(1.5)
    expect(p.yaw).toBe(0)
  })

  it('follows a rotated origin', () => {
    const meta = { ...META, origin: { x: 1, y: 2, yaw: Math.PI / 2 } }
    const p = mapPlacement(meta)
    const c = mapPixelToWorld(meta, 100, 50)
    expect(p.center.x).toBeCloseTo(c.x)
    expect(p.center.y).toBeCloseTo(c.y)
    expect(p.yaw).toBeCloseTo(Math.PI / 2)
  })
})

describe('mapBounds', () => {
  it('returns the axis-aligned extent of the map', () => {
    const b = mapBounds(META)
    expect(b.minX).toBeCloseTo(-3)
    expect(b.maxX).toBeCloseTo(7)
    expect(b.minY).toBeCloseTo(-1)
    expect(b.maxY).toBeCloseTo(4)
  })
})

describe('fitDistance', () => {
  it('distance so that a span fills the vertical field of view', () => {
    // fov 90 deg: half-height = distance
    expect(fitDistance(10, 10, 90, 1, 1)).toBeCloseTo(5)
  })

  it('uses the horizontal fov when the span is wider than the aspect', () => {
    expect(fitDistance(20, 1, 90, 2, 1)).toBeCloseTo(5)
  })

  it('never returns less than a minimum distance', () => {
    expect(fitDistance(0, 0, 50, 1)).toBeGreaterThan(0)
  })
})

describe('classifyBaseLink', () => {
  it('splits base URDF links into wheels and body', () => {
    expect(classifyBaseLink('fl_wheel_link')).toBe('wheels')
    expect(classifyBaseLink('rr_steer_link')).toBe('wheels')
    expect(classifyBaseLink('base_link')).toBe('body')
    expect(classifyBaseLink('lidar_link')).toBe('body')
  })
})
