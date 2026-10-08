import { describe, expect, it } from 'vitest'
import { formatUpdated, objectDetails, OBJECT_COLOR, parsePoiList, poiStyle, sortPois, STATUS_COLORS } from './style'
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

describe('object POIs', () => {
  const cup = { kind: 'object' as const, name: 'red cup', times_seen: 3, first_seen: 100, last_seen: 160, confidence: 0.8 }

  it('are drawn in their own colour, still marked by creator and faded when closed', () => {
    expect(poiStyle(poi(cup), false).fill).toBe(OBJECT_COLOR)
    expect(Object.values(STATUS_COLORS)).not.toContain(OBJECT_COLOR)
    expect(poiStyle(poi({ ...cup, created_by: 'agent' }), false).dashed).toBe(true)
    expect(poiStyle(poi({ ...cup, status: 'done' }), false).opacity).toBeLessThan(poiStyle(poi(cup), false).opacity)
  })

  it('are parsed with their sighting fields and default them when the store omits them', () => {
    const list = parsePoiList({ pois: [poi(cup), poi({ id: 'c'.repeat(32), kind: 'object' })], revision: 1 })
    expect(list?.pois).toHaveLength(2)
    expect(list?.pois[0]).toMatchObject({ times_seen: 3, last_seen: 160 })
    expect(list?.pois[1]).toMatchObject({ times_seen: 0, first_seen: 0, last_seen: 0, confidence: 1 })
    expect(parsePoiList({ pois: [poi({ kind: 'blob' as never })], revision: 1 })?.pois).toEqual([])
  })

  it('describe themselves for the tooltip and the list', () => {
    expect(objectDetails(poi(cup), 220)).toBe('seen 3x, last seen 1 min ago, confidence 80%')
    expect(objectDetails(poi({ ...cup, times_seen: 1, last_seen: 218 }), 220)).toBe('seen 1x, last seen just now, confidence 80%')
    expect(objectDetails(poi({ kind: 'object' }), 220)).toBe('')
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
