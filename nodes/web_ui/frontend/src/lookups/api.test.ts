import { describe, expect, it, vi } from 'vitest'
import { fetchLookups, lookupImageUrl, lookupsUrl } from './api'

function fakeFetch(status: number, body: unknown) {
  return vi.fn(async () => ({ ok: status < 400, status, json: async () => body }) as Response)
}

describe('lookupsUrl', () => {
  it('has only the limit without a POI', () => {
    expect(lookupsUrl(null, 8)).toBe('/api/lookups?limit=8')
  })

  it('filters by the selected POI (id is URL-encoded)', () => {
    expect(lookupsUrl('poi 1/x', 5)).toBe('/api/lookups?limit=5&poi_id=poi%201%2Fx')
  })
})

describe('lookupImageUrl', () => {
  it('points at the jpg route', () => {
    expect(lookupImageUrl('abc123')).toBe('/api/lookups/abc123.jpg')
  })
})

describe('fetchLookups', () => {
  it('returns the lookups of the backend, newest first as sent', async () => {
    const body = { lookups: [{ id: 'b', ts: 2, time: 't2', camera: 'head', tool: 'get_camera_image', note: 'n', poi_ids: ['p'], mcp_session: null }, { id: 'a', ts: 1, time: 't1', camera: 'gripper', tool: 'x', note: '', poi_ids: [] }] }
    const f = fakeFetch(200, body)
    const out = await fetchLookups('p', 10, f as unknown as typeof fetch)
    expect(out.map((l) => l.id)).toEqual(['b', 'a'])
    expect(out[0]).toMatchObject({ camera: 'head', note: 'n', poiIds: ['p'], time: 't2' })
    expect((f.mock.calls[0] as unknown as [string])[0]).toBe('/api/lookups?limit=10&poi_id=p')
  })

  it('never throws: HTTP errors, bad bodies and network failures give an empty list', async () => {
    expect(await fetchLookups(null, 5, fakeFetch(404, {}) as unknown as typeof fetch)).toEqual([])
    expect(await fetchLookups(null, 5, fakeFetch(200, { nope: 1 }) as unknown as typeof fetch)).toEqual([])
    const boom = vi.fn(async () => {
      throw new Error('offline')
    })
    expect(await fetchLookups(null, 5, boom as unknown as typeof fetch)).toEqual([])
  })
})
