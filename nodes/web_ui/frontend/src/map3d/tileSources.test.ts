import { describe, expect, it } from 'vitest'
import {
  TILE_SOURCE_STORAGE_KEY,
  cropUvs,
  overzoomTile,
  pickTileSource,
  readStoredTileSource,
  writeStoredTileSource,
  type TileSourceInfo,
} from './tileSources'

const SOURCES: TileSourceInfo[] = [
  { id: 'satellite', label: 'Satellite', max_zoom: 20, attribution: 'Esri', version: 'aa' },
  { id: 'street', label: 'Street', max_zoom: 18, attribution: 'CARTO', version: 'bb' },
]

describe('overzoomTile', () => {
  it('uses the tile itself at or below max zoom', () => {
    expect(overzoomTile(18, 10, 20, 18)).toEqual({ z: 18, x: 10, y: 20, u0: 0, v0: 0, u1: 1, v1: 1 })
    expect(overzoomTile(17, 5, 6, 18)).toEqual({ z: 17, x: 5, y: 6, u0: 0, v0: 0, u1: 1, v1: 1 })
  })

  it('z19 from z18 is a 2x2 quadrant of the parent', () => {
    expect(overzoomTile(19, 20, 40, 18)).toEqual({ z: 18, x: 10, y: 20, u0: 0, v0: 0, u1: 0.5, v1: 0.5 })
    expect(overzoomTile(19, 21, 40, 18)).toEqual({ z: 18, x: 10, y: 20, u0: 0.5, v0: 0, u1: 1, v1: 0.5 })
    expect(overzoomTile(19, 20, 41, 18)).toEqual({ z: 18, x: 10, y: 20, u0: 0, v0: 0.5, u1: 0.5, v1: 1 })
    expect(overzoomTile(19, 21, 41, 18)).toEqual({ z: 18, x: 10, y: 20, u0: 0.5, v0: 0.5, u1: 1, v1: 1 })
  })

  it('z20 from z18 is a 4x4 sub-grid of the parent', () => {
    // children of parent (10, 20) at z20 span x 40..43, y 80..83
    expect(overzoomTile(20, 40, 80, 18)).toEqual({ z: 18, x: 10, y: 20, u0: 0, v0: 0, u1: 0.25, v1: 0.25 })
    expect(overzoomTile(20, 43, 83, 18)).toEqual({ z: 18, x: 10, y: 20, u0: 0.75, v0: 0.75, u1: 1, v1: 1 })
    expect(overzoomTile(20, 42, 81, 18)).toEqual({ z: 18, x: 10, y: 20, u0: 0.5, v0: 0.25, u1: 0.75, v1: 0.5 })
  })
})

describe('cropUvs', () => {
  it('maps the full tile to the unit square (plane vertex order: top-left, top-right, bottom-left, bottom-right)', () => {
    expect(cropUvs({ u0: 0, v0: 0, u1: 1, v1: 1 })).toEqual([0, 1, 1, 1, 0, 0, 1, 0])
  })

  it('maps a crop from the top-left quadrant to the upper-left texture region (texture v is bottom-up)', () => {
    expect(cropUvs({ u0: 0, v0: 0, u1: 0.5, v1: 0.5 })).toEqual([0, 1, 0.5, 1, 0, 0.5, 0.5, 0.5])
  })

  it('maps the bottom-right quadrant to the lower-right texture region', () => {
    expect(cropUvs({ u0: 0.5, v0: 0.5, u1: 1, v1: 1 })).toEqual([0.5, 0.5, 1, 0.5, 0.5, 0, 1, 0])
  })
})

describe('pickTileSource', () => {
  it('prefers the stored choice when it exists', () => {
    expect(pickTileSource(SOURCES, 'street', 'satellite')?.id).toBe('street')
  })

  it('falls back to the default, then the first source', () => {
    expect(pickTileSource(SOURCES, 'gone', 'street')?.id).toBe('street')
    expect(pickTileSource(SOURCES, null, 'gone')?.id).toBe('satellite')
    expect(pickTileSource(SOURCES, null, null)?.id).toBe('satellite')
  })

  it('returns null without sources', () => {
    expect(pickTileSource([], 'street', 'street')).toBeNull()
  })
})

describe('stored tile source choice', () => {
  it('round-trips through storage', () => {
    const data = new Map<string, string>()
    const storage = { getItem: (k: string) => data.get(k) ?? null, setItem: (k: string, v: string) => void data.set(k, v) }
    writeStoredTileSource(storage, 'street')
    expect(data.get(TILE_SOURCE_STORAGE_KEY)).toBe('street')
    expect(readStoredTileSource(storage)).toBe('street')
  })

  it('survives missing or throwing storage', () => {
    const boom = {
      getItem: () => {
        throw new Error('blocked')
      },
      setItem: () => {
        throw new Error('blocked')
      },
    }
    expect(readStoredTileSource(boom)).toBeNull()
    expect(() => writeStoredTileSource(boom, 'street')).not.toThrow()
    expect(readStoredTileSource(undefined)).toBeNull()
    expect(() => writeStoredTileSource(undefined, 'street')).not.toThrow()
  })
})
