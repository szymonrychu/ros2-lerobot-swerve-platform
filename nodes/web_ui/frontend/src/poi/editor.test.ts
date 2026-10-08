import { describe, expect, it } from 'vitest'
import {
  addVertex,
  applyOverride,
  canFinishArea,
  CLICK_MAX_DRAG_PX,
  defaultName,
  dragCommand,
  dragPreview,
  isClick,
  newAreaFields,
  newPointFields,
  startDrag,
  updateDrag,
} from './editor'
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
    x: 1,
    y: 1,
    polygon: [],
    radius_m: 0.2,
    name: 'n',
    note: '',
    status: 'open',
    created_by: 'user',
    created_at: 0,
    updated_at: 0,
    ...over,
  }
}

describe('click detection', () => {
  it('is a click below the movement threshold only', () => {
    expect(isClick({ x: 10, y: 10 }, { x: 10 + CLICK_MAX_DRAG_PX - 1, y: 10 })).toBe(true)
    expect(isClick({ x: 10, y: 10 }, { x: 10 + CLICK_MAX_DRAG_PX + 1, y: 10 })).toBe(false)
  })
})

describe('area drafting', () => {
  it('appends vertices but drops one on top of the previous (double click)', () => {
    let d = addVertex([], { x: 0, y: 0 })
    d = addVertex(d, { x: 1, y: 0 })
    d = addVertex(d, { x: 1.001, y: 0.001 })
    expect(d).toHaveLength(2)
  })

  it('needs three vertices to finish', () => {
    expect(canFinishArea([{ x: 0, y: 0 }, { x: 1, y: 0 }])).toBe(false)
    expect(canFinishArea([{ x: 0, y: 0 }, { x: 1, y: 0 }, { x: 1, y: 1 }])).toBe(true)
  })

  it('builds the new-POI fields for a point and an area (centroid computed, created by the user)', () => {
    const p = newPointFields({ x: 1, y: 2 }, 'Point 1')
    expect(p).toMatchObject({ kind: 'point', x: 1, y: 2, radius_m: 0.2, name: 'Point 1', status: 'open', created_by: 'user' })
    const a = newAreaFields(SQUARE.map(([x, y]) => ({ x, y })), 'Area 1')
    expect(a.kind).toBe('area')
    expect(a.polygon).toEqual(SQUARE)
    expect(a.x).toBeCloseTo(1)
    expect(a.y).toBeCloseTo(1)
  })

  it('numbers default names by kind', () => {
    expect(defaultName('point', [])).toBe('Point 1')
    expect(defaultName('point', [poi({}), poi({ kind: 'area', polygon: SQUARE })])).toBe('Point 2')
    expect(defaultName('area', [poi({})])).toBe('Area 1')
  })
})

describe('dragging', () => {
  it('moves an object like a point', () => {
    const o = { ...poi({ x: 1, y: 1 }), kind: 'object' as const }
    const d = updateDrag(startDrag({ id: o.id, part: 'body' }, { x: 1, y: 1 }, { x: 0, y: 0 }), { x: 3, y: 4 }, { x: 40, y: 0 })
    expect(dragPreview(o, d)).toMatchObject({ x: 3, y: 4, kind: 'object' })
    expect(dragCommand(o, d)).toEqual({ op: 'update', poi: { id: o.id, x: 3, y: 4 } })
  })

  it('moves a point by the pointer and tracks the screen distance', () => {
    const p = poi({ x: 1, y: 1 })
    let d = startDrag({ id: p.id, part: 'body' }, { x: 1, y: 1 }, { x: 100, y: 100 })
    d = updateDrag(d, { x: 3, y: 4 }, { x: 130, y: 100 })
    expect(d.movedPx).toBe(30)
    expect(dragPreview(p, d)).toMatchObject({ x: 3, y: 4 })
    expect(dragCommand(p, d)).toEqual({ op: 'update', poi: { id: p.id, x: 3, y: 4 } })
  })

  it('keeps the grab offset when dragging a point by its body', () => {
    const p = poi({ x: 1, y: 1 })
    let d = startDrag({ id: p.id, part: 'body' }, { x: 1.1, y: 1 }, { x: 0, y: 0 })
    d = updateDrag(d, { x: 2.1, y: 1 }, { x: 50, y: 0 })
    expect(dragPreview(p, d)).toMatchObject({ x: expect.closeTo(2), y: expect.closeTo(1) })
  })

  it('moves one area vertex and recomputes the centroid', () => {
    const a = poi({ kind: 'area', polygon: SQUARE, x: 1, y: 1 })
    let d = startDrag({ id: a.id, part: 'vertex', index: 2 }, { x: 2, y: 2 }, { x: 0, y: 0 })
    d = updateDrag(d, { x: 4, y: 4 }, { x: 40, y: 0 })
    const prev = dragPreview(a, d)
    expect(prev.polygon[2]).toEqual([4, 4])
    expect(prev.polygon[0]).toEqual([0, 0])
    expect(prev.x).not.toBe(1)
    expect(dragCommand(a, d)).toEqual({ op: 'update', poi: { id: a.id, polygon: prev.polygon } })
  })

  it('translates a whole area when dragged by its body', () => {
    const a = poi({ kind: 'area', polygon: SQUARE, x: 1, y: 1 })
    let d = startDrag({ id: a.id, part: 'body' }, { x: 1, y: 1 }, { x: 0, y: 0 })
    d = updateDrag(d, { x: 2, y: 3 }, { x: 40, y: 0 })
    const prev = dragPreview(a, d)
    expect(prev.polygon).toEqual([
      [1, 2],
      [3, 2],
      [3, 4],
      [1, 4],
    ])
    expect(prev.x).toBeCloseTo(2)
    expect(prev.y).toBeCloseTo(3)
  })

  it('ignores a pick that misses the ground', () => {
    const d = startDrag({ id: 'x', part: 'body' }, { x: 1, y: 1 }, { x: 0, y: 0 })
    expect(updateDrag(d, null, { x: 5, y: 0 }).current).toEqual({ x: 1, y: 1 })
  })
})

describe('overrides', () => {
  it('replaces the matching POI and leaves the others', () => {
    const a = poi({ id: '1'.repeat(32) })
    const b = poi({ id: '2'.repeat(32) })
    const moved = { ...a, x: 9 }
    expect(applyOverride([a, b], moved)).toEqual([moved, b])
    expect(applyOverride([a, b], null)).toEqual([a, b])
  })
})
