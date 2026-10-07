import { describe, expect, it } from 'vitest'
import { formatUpdated, parsePoiList, poiStyle, sortPois, STATUS_COLORS } from './style'
import type { Poi } from './types'

function poi(over: Partial<Poi>): Poi {
  return {
    id: 'a'.repeat(32),
    kind: 'point',
    frame: 'map',
    x: 0,
    y: 0,
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

describe('poiStyle', () => {
  it('colours by status and marks the creator', () => {
    expect(poiStyle(poi({ status: 'open' }), false).fill).toBe(STATUS_COLORS.open)
    expect(poiStyle(poi({ status: 'done' }), false).fill).toBe(STATUS_COLORS.done)
    expect(poiStyle(poi({ status: 'cancelled' }), false).fill).toBe(STATUS_COLORS.cancelled)
    expect(new Set(Object.values(STATUS_COLORS)).size).toBe(3)
    const agent = poiStyle(poi({ created_by: 'agent' }), false)
    const user = poiStyle(poi({ created_by: 'user' }), false)
    expect(agent.creatorTag).toBe('agent')
    expect(user.creatorTag).toBe('user')
    expect(agent.outline).not.toBe(user.outline)
    expect(agent.dashed).toBe(true)
    expect(user.dashed).toBe(false)
  })

  it('fades closed POIs and thickens the selected outline', () => {
    expect(poiStyle(poi({ status: 'cancelled' }), false).opacity).toBeLessThan(poiStyle(poi({ status: 'open' }), false).opacity)
    expect(poiStyle(poi({}), true).lineWidth).toBeGreaterThan(poiStyle(poi({}), false).lineWidth)
  })
})

describe('parsePoiList', () => {
  it('accepts a valid list and keeps the revision', () => {
    const list = parsePoiList({ pois: [poi({})], revision: 3 })
    expect(list?.revision).toBe(3)
    expect(list?.pois).toHaveLength(1)
  })

  it('drops malformed entries and rejects non-lists', () => {
    const list = parsePoiList({
      pois: [poi({}), { id: 'x' }, poi({ id: 'b'.repeat(32), kind: 'area', polygon: [[0, 0], [1, 1]] }), null],
      revision: 1,
    })
    expect(list?.pois.map((p) => p.id)).toEqual(['a'.repeat(32)])
    expect(parsePoiList(null)).toBeNull()
    expect(parsePoiList({ pois: 'x', revision: 1 })).toBeNull()
    expect(parsePoiList(undefined)).toBeNull()
  })
})

describe('list helpers', () => {
  it('sorts by updated_at descending without mutating', () => {
    const a = poi({ id: '1'.repeat(32), updated_at: 1 })
    const b = poi({ id: '2'.repeat(32), updated_at: 5 })
    const input = [a, b]
    expect(sortPois(input).map((p) => p.id)).toEqual([b.id, a.id])
    expect(input[0]).toBe(a)
  })

  it('formats the update time relative to now', () => {
    expect(formatUpdated(100, 130)).toBe('30 s ago')
    expect(formatUpdated(100, 100 + 600)).toBe('10 min ago')
    expect(formatUpdated(100, 100 + 7200)).toBe('2 h ago')
    expect(formatUpdated(100, 100 + 3 * 86400)).toBe('3 d ago')
    expect(formatUpdated(200, 100)).toBe('just now')
  })
})
