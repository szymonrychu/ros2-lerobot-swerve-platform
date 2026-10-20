import { afterEach, describe, expect, it, vi } from 'vitest'
import { fetchServoGroups, servoAction } from './servoApi'

afterEach(() => {
  vi.unstubAllGlobals()
})

function respond(status: number, body: unknown): void {
  vi.stubGlobal('fetch', vi.fn(async () => new Response(JSON.stringify(body), { status })))
}

describe('servoApi', () => {
  it('fetchServoGroups returns the groups', async () => {
    const group = { id: 'arm', label: 'Arm', neutral: true, state: null, age_s: null, stale: true, calibration_stale: null }
    respond(200, { groups: [group] })
    const r = await fetchServoGroups()
    expect(fetch).toHaveBeenCalledWith('/api/calibration/servos', { cache: 'no-store' })
    expect(r).toEqual({ ok: true, value: [group] })
  })

  it('404 is reported as status 404', async () => {
    respond(404, { message: 'not configured' })
    const r = await fetchServoGroups()
    expect(r).toEqual({ ok: false, status: 404, message: 'not configured' })
  })

  it('network errors carry status 0', async () => {
    vi.stubGlobal('fetch', vi.fn(async () => { throw new Error('link down') }))
    expect(await fetchServoGroups()).toEqual({ ok: false, status: 0, message: 'link down' })
  })

  it('malformed body is an error', async () => {
    respond(200, { nope: 1 })
    const r = await fetchServoGroups()
    expect(r.ok).toBe(false)
  })

  it.each(['release', 'enable', 'record', 'save'] as const)('servoAction posts %s', async (action) => {
    respond(200, { ok: true, message: 'done' })
    const r = await servoAction('arm', action)
    expect(fetch).toHaveBeenCalledWith(`/api/calibration/servos/arm/${action}`, { method: 'POST' })
    expect(r).toEqual({ ok: true, value: { ok: true, message: 'done' } })
  })

  it('a refused action (409) surfaces the backend message', async () => {
    respond(409, { ok: false, message: 'not released' })
    const r = await servoAction('head', 'record')
    expect(r).toEqual({ ok: false, status: 409, message: 'not released' })
  })
})
