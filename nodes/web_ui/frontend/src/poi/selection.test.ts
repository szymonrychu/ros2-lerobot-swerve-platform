import { describe, expect, it } from 'vitest'
import { filterPois, markAllVisible, poiCategory, pruneMarked, summariseMarked, toggleMarked, UNCATEGORISED } from './selection'
import type { Poi } from './types'

function mk(id: string, over: Partial<Poi> = {}): Poi {
  return {
    id, kind: 'point', frame: 'map', x: 0, y: 0, polygon: [], radius_m: 0.5, name: id, note: '',
    status: 'open', created_by: 'user', created_at: 0, updated_at: 0, ...over,
  }
}

const A = mk('a', { note: '#target door' })
const B = mk('b', { kind: 'area', zone: 'keepout' })
const C = mk('c', { kind: 'area', zone: 'slow' })
const D = mk('d')
const E = mk('e', { kind: 'area', zone: 'keepout', note: '#detail x' })
const ALL = [A, B, C, D, E]

describe('poiCategory', () => {
  it('prefers tag, then zone, else uncategorised', () => {
    expect(poiCategory(A)).toBe('target')
    expect(poiCategory(B)).toBe('keepout')
    expect(poiCategory(C)).toBe('slow')
    expect(poiCategory(D)).toBe(UNCATEGORISED)
    expect(poiCategory(E)).toBe('detail')
  })
})

describe('filterPois', () => {
  it('empty set passes everything', () => {
    expect(filterPois(ALL, new Set())).toEqual(ALL)
  })
  it('filters by category', () => {
    expect(filterPois(ALL, new Set(['keepout', UNCATEGORISED])).map((p) => p.id)).toEqual(['b', 'd'])
  })
})

describe('marking', () => {
  it('toggleMarked adds and removes without mutating', () => {
    const m = new Set<string>()
    const m2 = toggleMarked(m, 'a')
    expect([...m2]).toEqual(['a'])
    expect(m.size).toBe(0)
    expect(toggleMarked(m2, 'a').size).toBe(0)
  })
  it('markAllVisible marks exactly the visible ids', () => {
    expect([...markAllVisible(filterPois(ALL, new Set(['slow', 'target'])))].sort()).toEqual(['a', 'c'])
  })
  it('pruneMarked drops hidden and missing ids', () => {
    const visible = filterPois(ALL, new Set(['target']))
    expect([...pruneMarked(new Set(['a', 'b', 'zzz']), visible)]).toEqual(['a'])
  })
})

describe('summariseMarked', () => {
  it('counts, groups and finds keepouts by zone', () => {
    const s = summariseMarked(ALL, new Set(['a', 'b', 'd', 'e', 'gone']))
    expect(s.count).toBe(4)
    expect(s.byCategory).toEqual({ target: 1, keepout: 1, [UNCATEGORISED]: 1, detail: 1 })
    expect(s.keepouts.map((p) => p.id)).toEqual(['b', 'e'])
  })
})
