import { describe, expect, it } from 'vitest'
import { designViolations, SOURCES } from '../layout/designRules'
import { formatUpdated, KIND_COLORS, KIND_SHAPE, markerColors, markerSides, ZONE_COLORS, objectDetails, OBJECT_COLOR, parsePoiList, poiStyle, sortPois, STATUS_COLORS, zoneLabel, zoneStyle } from './style'
import type { Poi, PoiKind } from './types'

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

const SQUARE: [number, number][] = [
  [0, 0],
  [1, 0],
  [1, 1],
]

function area(over: Partial<Poi>): Poi {
  return poi({ kind: 'area', polygon: SQUARE, ...over })
}

describe('parsePoiList zones', () => {
  const parse = (p: unknown) => parsePoiList({ pois: [p], revision: 1 })?.pois ?? []

  it('defaults a missing zone to none', () => {
    const [p] = parse(area({}))
    expect(p.zone).toBe('none')
  })

  it('keeps valid keepout and slow zones', () => {
    expect(parse(area({ zone: 'keepout', speed_limit_pct: null }))[0].zone).toBe('keepout')
    const [slow] = parse(area({ zone: 'slow', speed_limit_pct: 40 }))
    expect(slow.zone).toBe('slow')
    expect(slow.speed_limit_pct).toBe(40)
  })

  it('drops an unknown zone', () => {
    expect(parse(area({ zone: 'lava' as never }))).toHaveLength(0)
  })

  it('drops a slow zone without a valid pct', () => {
    expect(parse(area({ zone: 'slow' }))).toHaveLength(0)
    expect(parse(area({ zone: 'slow', speed_limit_pct: null }))).toHaveLength(0)
    expect(parse(area({ zone: 'slow', speed_limit_pct: 5 }))).toHaveLength(0)
    expect(parse(area({ zone: 'slow', speed_limit_pct: 101 }))).toHaveLength(0)
    expect(parse(area({ zone: 'slow', speed_limit_pct: 40.5 }))).toHaveLength(0)
    expect(parse(area({ zone: 'slow', speed_limit_pct: 10 }))).toHaveLength(1)
    expect(parse(area({ zone: 'slow', speed_limit_pct: 100 }))).toHaveLength(1)
  })
})

describe('zoneStyle and zoneLabel', () => {
  it('is null / empty for a plain area', () => {
    expect(zoneStyle(area({}), false)).toBeNull()
    expect(zoneStyle(area({ zone: 'none' }), false)).toBeNull()
    expect(zoneLabel(area({ zone: 'none' }))).toBe('')
  })

  it('draws an open keepout red, thick and solid', () => {
    const s = zoneStyle(area({ zone: 'keepout' }), false)
    expect(s).toMatchObject({ fill: ZONE_COLORS.keepout, outline: ZONE_COLORS.keepout, opacity: 0.45, dashed: false })
    expect(s?.lineWidth).toBeGreaterThanOrEqual(4)
  })

  it('draws an open slow zone amber', () => {
    const s = zoneStyle(area({ zone: 'slow', speed_limit_pct: 40 }), false)
    expect(s).toMatchObject({ fill: ZONE_COLORS.slow, opacity: 0.35 })
  })

  it('greys out a zone whose status is not open', () => {
    for (const status of ['done', 'cancelled'] as const) {
      const s = zoneStyle(area({ zone: 'keepout', status }), false)
      expect(s?.fill).toBe(STATUS_COLORS.cancelled)
      expect(s?.opacity).toBeLessThan(0.35)
    }
  })

  it('keeps the selection outline', () => {
    expect(zoneStyle(area({ zone: 'keepout' }), true)?.outline).not.toBe(ZONE_COLORS.keepout)
  })

  it('labels zones', () => {
    expect(zoneLabel(area({ zone: 'keepout' }))).toBe('KEEPOUT')
    expect(zoneLabel(area({ zone: 'slow', speed_limit_pct: 40 }))).toBe('SLOW 40%')
  })
})

describe('markerColors', () => {
  it('has kind colours that never reuse a status colour', () => {
    expect(Object.keys(KIND_COLORS).sort()).toEqual(['area-none', 'object', 'photo', 'point'])
    for (const c of Object.values(KIND_COLORS)) expect(Object.values(STATUS_COLORS)).not.toContain(c)
    expect(new Set(Object.values(KIND_COLORS)).size).toBe(4)
  })

  it('fills points and objects by kind and outlines them by status', () => {
    expect(markerColors(poi({ kind: 'point', status: 'done' }))).toEqual({ fill: KIND_COLORS.point, outline: STATUS_COLORS.done })
    expect(markerColors(poi({ kind: 'object', status: 'open' }))).toEqual({ fill: KIND_COLORS.object, outline: STATUS_COLORS.open })
    expect(KIND_COLORS.object).toBe(OBJECT_COLOR)
  })

  it('colours areas by zone: slow amber, keepout red, none neutral', () => {
    const area = { kind: 'area' as const, polygon: [[0, 0], [1, 0], [0, 1]] as [number, number][] }
    expect(markerColors(poi({ ...area, zone: 'keepout' }))).toEqual({ fill: ZONE_COLORS.keepout, outline: ZONE_COLORS.keepout })
    expect(markerColors(poi({ ...area, zone: 'slow', speed_limit_pct: 40 }))).toEqual({ fill: ZONE_COLORS.slow, outline: ZONE_COLORS.slow })
    expect(markerColors(poi({ ...area, zone: 'none' }))).toEqual({ fill: KIND_COLORS['area-none'], outline: KIND_COLORS['area-none'] })
    expect(ZONE_COLORS.keepout).not.toBe(ZONE_COLORS.slow)
  })
})

describe('KIND_SHAPE', () => {
  const kinds: PoiKind[] = ['point', 'area', 'object', 'photo']

  it('gives every kind its own shape', () => {
    expect(Object.keys(KIND_SHAPE).sort()).toEqual([...kinds].sort())
    expect(new Set(kinds.map((k) => KIND_SHAPE[k])).size).toBe(kinds.length)
  })

  it('draws each kind with the polygon sides of its shape', () => {
    expect(markerSides('point')).toBeGreaterThan(8)
    expect(markerSides('object')).toBe(4)
    expect(markerSides('area')).toBe(4)
    expect(markerSides('photo')).toBe(3)
  })

  it('has PoiLayer take the sides of the point and object markers from their kind shape', () => {
    const layer = SOURCES['poi/PoiLayer.tsx']
    expect(layer).toContain("markerSides('point')")
    expect(layer).toContain("markerSides('object')")
    expect(layer).not.toContain('SHAPE_SIDES')
  })

  it('keeps the poi marker sources free of design-rule violations', () => {
    for (const path of ['poi/style.ts', 'poi/PoiLayer.tsx', 'poi/PoiLegend.tsx']) {
      expect(SOURCES[path], path).toBeTruthy()
      expect(designViolations(path, SOURCES[path])).toEqual([])
    }
  })
})
