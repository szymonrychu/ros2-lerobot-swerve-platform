import { describe, expect, it } from 'vitest'
import { hitTest, pointInPolygon, polygonArea, polygonCentroid, distance, nearestVertex } from './geometry'
import type { Poi } from './types'

const SQUARE: [number, number][] = [
  [0, 0],
  [2, 0],
  [2, 2],
  [0, 2],
]

function poi(over: Partial<Poi>): Poi {
  return {
    id: 'a'.repeat(32),
    kind: 'point',
    frame: 'map',
    x: 0,
    y: 0,
    polygon: [],
    radius_m: 0.2,
    name: '',
    note: '',
    status: 'open',
    created_by: 'user',
    created_at: 0,
    updated_at: 0,
    ...over,
  }
}

describe('polygon helpers', () => {
  it('computes area (absolute) and centroid', () => {
    expect(polygonArea(SQUARE)).toBeCloseTo(4)
    expect(polygonArea([...SQUARE].reverse())).toBeCloseTo(4)
    expect(polygonCentroid(SQUARE)).toEqual({ x: expect.closeTo(1), y: expect.closeTo(1) })
  })

  it('centroid of an L-shape is area weighted, not the vertex mean', () => {
    const l: [number, number][] = [
      [0, 0],
      [3, 0],
      [3, 1],
      [1, 1],
      [1, 3],
      [0, 3],
    ]
    const c = polygonCentroid(l)
    expect(c.x).toBeCloseTo(1.1, 5)
    expect(c.y).toBeCloseTo(1.1, 5)
  })

  it('falls back to the vertex mean for a degenerate polygon', () => {
    expect(polygonCentroid([[0, 0], [2, 0], [4, 0]])).toEqual({ x: 2, y: 0 })
  })

  it('tests point containment', () => {
    expect(pointInPolygon({ x: 1, y: 1 }, SQUARE)).toBe(true)
    expect(pointInPolygon({ x: 3, y: 1 }, SQUARE)).toBe(false)
    expect(pointInPolygon({ x: 1, y: 1 }, [[0, 0], [1, 1]])).toBe(false)
  })

  it('measures distance and finds the nearest vertex within a tolerance', () => {
    expect(distance({ x: 0, y: 0 }, { x: 3, y: 4 })).toBe(5)
    expect(nearestVertex(SQUARE, { x: 2.05, y: 0.02 }, 0.1)).toBe(1)
    expect(nearestVertex(SQUARE, { x: 1, y: 1 }, 0.1)).toBe(-1)
  })
})

describe('hitTest', () => {
  it('hits a point inside its radius or the tolerance, whichever is larger', () => {
    const p = poi({ x: 5, y: 5, radius_m: 0.2 })
    expect(hitTest([p], { x: 5.15, y: 5 }, 0.05, null)).toEqual({ id: p.id, part: 'body' })
    expect(hitTest([p], { x: 5.3, y: 5 }, 0.05, null)).toBeNull()
    expect(hitTest([p], { x: 5.3, y: 5 }, 0.4, null)?.id).toBe(p.id)
  })

  it('hits an area by its interior', () => {
    const a = poi({ id: 'b'.repeat(32), kind: 'area', polygon: SQUARE, x: 1, y: 1 })
    expect(hitTest([a], { x: 1.5, y: 0.5 }, 0.05, null)).toEqual({ id: a.id, part: 'body' })
    expect(hitTest([a], { x: 5, y: 5 }, 0.05, null)).toBeNull()
  })

  it('prefers the nearest point, then points over areas, and the smaller of overlapping areas', () => {
    const big = poi({ id: '1'.repeat(32), kind: 'area', polygon: [[-5, -5], [5, -5], [5, 5], [-5, 5]] })
    const small = poi({ id: '2'.repeat(32), kind: 'area', polygon: SQUARE })
    const near = poi({ id: '3'.repeat(32), x: 1, y: 1 })
    const nearer = poi({ id: '4'.repeat(32), x: 1.05, y: 1 })
    expect(hitTest([big, small], { x: 1, y: 1 }, 0.05, null)?.id).toBe(small.id)
    expect(hitTest([big, small, near], { x: 1, y: 1 }, 0.05, null)?.id).toBe(near.id)
    expect(hitTest([near, nearer], { x: 1.04, y: 1 }, 0.05, null)?.id).toBe(nearer.id)
  })

  it('hits a vertex of the selected area first, and ignores vertices of unselected areas', () => {
    const a = poi({ id: 'b'.repeat(32), kind: 'area', polygon: SQUARE })
    expect(hitTest([a], { x: 1.95, y: 1.95 }, 0.1, a.id)).toEqual({ id: a.id, part: 'vertex', index: 2 })
    expect(hitTest([a], { x: 1.95, y: 1.95 }, 0.1, null)).toEqual({ id: a.id, part: 'body' })
  })
})
