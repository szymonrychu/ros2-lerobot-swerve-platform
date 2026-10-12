import { describe, expect, it } from 'vitest'
import type { Photo } from '../photos/api'
import type { Poi } from '../poi/types'
import { resolveFocus } from './mapFocus'

function poi(over: Partial<Poi>): Poi {
  return {
    id: 'p1', kind: 'point', frame: 'map', x: 1, y: 2, polygon: [], radius_m: 0.5, name: 'Door', note: '',
    status: 'open', created_by: 'user', created_at: 1, updated_at: 1, ...over,
  }
}

function photo(over: Partial<Photo>): Photo {
  return {
    id: 'f1', ts: 1, camera: 'head', tool: 'take_photo', name: 'Shot', note: '', poiIds: [], x: 3, y: 4,
    view: null, bytes: 1, width: 1, height: 1, ...over,
  }
}

describe('resolveFocus', () => {
  it('finds a point POI', () => {
    expect(resolveFocus({ kind: 'poi', id: 'p1' }, [poi({})], [])).toEqual({ point: { x: 1, y: 2 }, poiId: 'p1', photo: null })
  })

  it('centres an area on its centroid', () => {
    const polygon: [number, number][] = [[0, 0], [4, 0], [4, 2], [0, 2]]
    const r = resolveFocus({ kind: 'poi', id: 'a' }, [poi({ id: 'a', kind: 'area', polygon })], [])
    expect(r?.point.x).toBeCloseTo(2)
    expect(r?.point.y).toBeCloseTo(1)
  })

  it('finds a located photo', () => {
    const p = photo({})
    expect(resolveFocus({ kind: 'photo', id: 'f1' }, [], [p])).toEqual({ point: { x: 3, y: 4 }, poiId: null, photo: p })
  })

  it('returns null for an unknown id or an unlocated photo', () => {
    expect(resolveFocus({ kind: 'poi', id: 'zz' }, [poi({})], [])).toBeNull()
    expect(resolveFocus({ kind: 'photo', id: 'f1' }, [], [photo({ x: null, y: null })])).toBeNull()
  })
})
