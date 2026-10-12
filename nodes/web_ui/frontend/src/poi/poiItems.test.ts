import { describe, expect, it } from 'vitest'
import type { Photo } from '../photos/api'
import { filterItems, focusRequestFor, isLocated, sortItems, toPoiItems, type PoiItem } from './poiItems'
import { sortPois } from './style'
import type { Poi } from './types'

function poi(over: Partial<Poi>): Poi {
  return {
    id: 'p1', kind: 'point', frame: 'map', x: 1, y: 2, polygon: [], radius_m: 0.5, name: 'Door', note: '',
    status: 'open', created_by: 'user', created_at: 100, updated_at: 100, ...over,
  }
}

function photo(over: Partial<Photo>): Photo {
  return {
    id: 'p1', ts: 100, camera: 'head', tool: 'take_photo', name: 'Shot', note: '', poiIds: [], x: 3, y: 4,
    view: null, bytes: 1, width: 1, height: 1, ...over,
  }
}

describe('toPoiItems', () => {
  it('maps a point, with status and zone', () => {
    const [item] = toPoiItems([poi({ zone: 'slow', note: 'n', updated_at: 7 })], [])
    expect(item).toMatchObject({ kind: 'point', name: 'Door', x: 1, y: 2, status: 'open', zone: 'slow', note: 'n', updated: 7, source: 'poi' })
  })

  it('uses the centroid of an area', () => {
    const polygon: [number, number][] = [[0, 0], [4, 0], [4, 2], [0, 2]]
    const [item] = toPoiItems([poi({ kind: 'area', polygon, x: 99, y: 99 })], [])
    expect(item.x).toBeCloseTo(2)
    expect(item.y).toBeCloseTo(1)
  })

  it('maps a photo and keeps null coordinates', () => {
    const items = toPoiItems([], [photo({ x: null, y: null, ts: 5 }), photo({ id: 'p2' })])
    expect(items[0]).toMatchObject({ kind: 'photo', x: null, y: null, updated: 5, source: 'photo' })
    expect(items[0].status).toBeUndefined()
    expect(items[1]).toMatchObject({ x: 3, y: 4 })
  })

  it('keys are unique across sources sharing an id', () => {
    const items = toPoiItems([poi({ id: 'a' })], [photo({ id: 'a' })])
    expect(new Set(items.map((i) => i.key)).size).toBe(2)
  })
})

describe('filterItems', () => {
  const items = toPoiItems(
    [poi({ id: 'a', name: 'Kitchen door', note: 'blue' }), poi({ id: 'b', kind: 'area', polygon: [[0, 0], [1, 0], [1, 1]], name: 'Rug' })],
    [photo({ id: 'c', name: 'Sofa', note: 'Kitchen view' })],
  )

  it('filters by kind', () => {
    expect(filterItems(items, new Set(['photo']), '').map((i) => i.name)).toEqual(['Sofa'])
  })

  it('treats an empty kind set as no kind filter', () => {
    expect(filterItems(items, new Set(), '')).toHaveLength(3)
  })

  it('searches name and note case-insensitively', () => {
    expect(filterItems(items, new Set(), 'KITCHEN').map((i) => i.name)).toEqual(['Kitchen door', 'Sofa'])
  })

  it('combines kind and query', () => {
    expect(filterItems(items, new Set(['point']), 'kitchen').map((i) => i.name)).toEqual(['Kitchen door'])
  })
})

describe('sortItems', () => {
  it('orders POIs like sortPois and photos by updated, newest first, without mutating', () => {
    const pois = [poi({ id: 'a', updated_at: 10 }), poi({ id: 'b', updated_at: 30 })]
    const items = toPoiItems(pois, [photo({ id: 'c', ts: 20 }), photo({ id: 'd', ts: 40 })])
    const before = [...items]
    const sorted = sortItems(items)
    expect(items).toEqual(before)
    expect(sorted.map((i) => i.key)).toEqual(['photo:d', 'poi:b', 'photo:c', 'poi:a'])
    expect(sorted.filter((i) => i.source === 'poi').map((i) => i.key)).toEqual(sortPois(pois).map((p) => `poi:${p.id}`))
  })
})

describe('isLocated / focusRequestFor', () => {
  const [located, unlocated] = toPoiItems([], [photo({ id: 'a' }), photo({ id: 'b', x: null, y: null })]) as [PoiItem, PoiItem]

  it('detects located items', () => {
    expect(isLocated(located)).toBe(true)
    expect(isLocated(unlocated)).toBe(false)
  })

  it('builds a focus request, null when unlocated', () => {
    expect(focusRequestFor(located)).toEqual({ kind: 'photo', id: 'a' })
    expect(focusRequestFor(unlocated)).toBeNull()
    expect(focusRequestFor(toPoiItems([poi({ id: 'z' })], [])[0])).toEqual({ kind: 'poi', id: 'z' })
  })
})
