import { describe, expect, it, vi } from 'vitest'
import { postGrasp, postGraspStop } from './graspApi'

function fakeFetch(status: number, body: unknown) {
  return vi.fn(async () => ({ status, json: async () => body }) as unknown as Response)
}

describe('postGrasp', () => {
  it('posts the request as JSON to the tab endpoint and parses the answer', async () => {
    const f = fakeFetch(200, { ok: true, request_id: 'r', action: 'plan', result: { outcome: 'planned', reasons: [] } })
    const out = await postGrasp('map tab', { action: 'plan' }, f)
    expect(f).toHaveBeenCalledWith('/api/grasp?tab=map%20tab', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ action: 'plan' }),
    })
    expect(out).toMatchObject({ ok: true, outcome: 'planned' })
  })

  it('returns the accepted reply of an execute', async () => {
    const out = await postGrasp('map', { action: 'execute' }, fakeFetch(202, { ok: true, accepted: true, request_id: 'r', action: 'execute' }))
    expect(out).toMatchObject({ ok: true, accepted: true })
  })

  it('maps a network failure to an error answer', async () => {
    const f = vi.fn(async () => {
      throw new Error('offline')
    })
    const out = await postGrasp('map', { action: 'plan' }, f)
    expect(out.ok).toBe(false)
    expect(out.error).toContain('offline')
  })

  it('maps a non-JSON body to an error answer', async () => {
    const f = vi.fn(async () => ({
      status: 502,
      json: async () => {
        throw new Error('bad json')
      },
    }) as unknown as Response)
    expect(await postGrasp('map', { action: 'plan' }, f)).toMatchObject({ ok: false, error: 'HTTP 502' })
  })
})

describe('postGraspStop', () => {
  it('posts to the stop endpoint', async () => {
    const f = fakeFetch(200, { ok: true, request_id: 'r', action: 'stop', result: { arm_held: true, message: 'held' } })
    const out = await postGraspStop('map', f)
    expect(f).toHaveBeenCalledWith('/api/grasp/stop?tab=map', { method: 'POST' })
    expect(out.ok).toBe(true)
  })

  it('reports a failed stop', async () => {
    expect(await postGraspStop('map', fakeFetch(503, { ok: false, message: 'mcp_server is not running' }))).toMatchObject({
      ok: false,
      error: 'mcp_server is not running',
    })
  })
})
