import { describe, expect, it } from 'vitest'
import { recordTileResult, tileFailureCaption, type TileResults } from './tileStatus'

const urls = ['a', 'b', 'c']

describe('tileFailureCaption', () => {
  it('is null while nothing failed (loading or loaded)', () => {
    expect(tileFailureCaption({}, urls)).toBeNull()
    expect(tileFailureCaption(recordTileResult({}, 'a', { ok: true }), urls)).toBeNull()
  })

  it('names the shared HTTP status when every settled tile failed with it', () => {
    let r: TileResults = {}
    for (const u of urls) r = recordTileResult(r, u, { ok: false, status: 403 })
    expect(tileFailureCaption(r, urls)).toBe('Map tiles failed to load (HTTP 403)')
  })

  it('says unavailable when all failed without a common status', () => {
    let r: TileResults = recordTileResult({}, 'a', { ok: false, status: 403 })
    r = recordTileResult(r, 'b', { ok: false, status: null })
    expect(tileFailureCaption(r, ['a', 'b'])).toBe('Map tiles unavailable')
  })

  it('reports a partial failure with counts', () => {
    let r: TileResults = recordTileResult({}, 'a', { ok: true })
    r = recordTileResult(r, 'b', { ok: false, status: 404 })
    expect(tileFailureCaption(r, urls)).toBe('1 of 2 map tiles failed to load')
  })

  it('only counts the given urls and a later success replaces a failure', () => {
    let r: TileResults = recordTileResult({}, 'old', { ok: false, status: 403 })
    expect(tileFailureCaption(r, ['a'])).toBeNull()
    r = recordTileResult(r, 'a', { ok: false, status: null })
    r = recordTileResult(r, 'a', { ok: true })
    expect(tileFailureCaption(r, ['a'])).toBeNull()
  })

  it('recordTileResult does not mutate its input', () => {
    const r: TileResults = {}
    recordTileResult(r, 'a', { ok: true })
    expect(r).toEqual({})
  })
})
