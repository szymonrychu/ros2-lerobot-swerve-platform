import { describe, expect, it, vi } from 'vitest'
import { postPoi } from './api'

function fakeFetch(status: number, body: unknown) {
  return vi.fn(async () => ({ status, json: async () => body }) as Response)
}

describe('postPoi', () => {
  it('posts the command as JSON to /api/poi with the tab id and returns the store result', async () => {
    const result = { request_id: 'r', ok: true, message: 'added', poi: { id: 'a' } }
    const f = fakeFetch(200, result)
    const out = await postPoi('map tab', { op: 'add', poi: { kind: 'point' } }, f as unknown as typeof fetch)
    expect(out).toEqual({ ok: true, message: 'added', poi: { id: 'a' } })
    const [url, init] = f.mock.calls[0] as unknown as [string, RequestInit]
    expect(url).toBe('/api/poi?tab=map%20tab')
    expect(init.method).toBe('POST')
    expect(JSON.parse(init.body as string)).toEqual({ op: 'add', poi: { kind: 'point' } })
  })

  it('reports store rejections and backend errors with their message', async () => {
    expect(await postPoi('m', { op: 'delete', poi: { id: 'x' } }, fakeFetch(400, { ok: false, message: 'unknown poi id x', poi: null }) as unknown as typeof fetch)).toEqual({
      ok: false,
      message: 'unknown poi id x',
      poi: null,
    })
    const down = await postPoi('m', { op: 'add', poi: {} }, fakeFetch(503, { ok: false, message: 'poi_store is not running' }) as unknown as typeof fetch)
    expect(down.ok).toBe(false)
    expect(down.message).toBe('poi_store is not running')
  })

  it('never throws: network failures and non-JSON bodies become an error result', async () => {
    const boom = vi.fn(async () => {
      throw new Error('offline')
    })
    expect((await postPoi('m', { op: 'add', poi: {} }, boom as unknown as typeof fetch)).message).toContain('offline')
    const bad = vi.fn(async () => ({ status: 502, json: async () => { throw new Error('not json') } }) as unknown as Response)
    const out = await postPoi('m', { op: 'add', poi: {} }, bad as unknown as typeof fetch)
    expect(out.ok).toBe(false)
    expect(out.message).toContain('502')
  })
})
