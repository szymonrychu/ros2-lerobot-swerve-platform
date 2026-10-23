// @vitest-environment jsdom
import { afterEach, describe, expect, it, vi } from 'vitest'
import { applyFloor, computeFloor, getFloorResult, getFloorState, startSweep, stopSweep } from './headFloorApi'
import stateFx from '../../../../head_rgbd_calibration/tests/fixtures/api/floor_state.json'
import resultFx from '../../../../head_rgbd_calibration/tests/fixtures/api/floor_result.json'
import applyFx from '../../../../head_rgbd_calibration/tests/fixtures/api/floor_apply.json'

const B = '/api/calibration/rgbd/floor'

function stub(status: number, body: unknown) {
  const f = vi.fn(async () => new Response(JSON.stringify(body), { status }))
  vi.stubGlobal('fetch', f)
  return f
}
afterEach(() => {
  vi.unstubAllGlobals()
})

describe('headFloorApi', () => {
  it('getFloorState accepts the node fixture', async () => {
    const f = stub(200, stateFx)
    expect(await getFloorState()).toEqual({ ok: true, value: stateFx })
    expect(f).toHaveBeenCalledWith(`${B}/state`, { cache: 'no-store' })
  })
  it('getFloorState accepts applied null', async () => {
    stub(200, { sweep: stateFx.sweep, applied: null })
    expect((await getFloorState()).ok).toBe(true)
  })
  it('getFloorState rejects a malformed applied source or floor', async () => {
    stub(200, { sweep: stateFx.sweep, applied: { ...stateFx.applied, source: 7 } })
    expect((await getFloorState()).ok).toBe(false)
    stub(200, { sweep: stateFx.sweep, applied: { ...stateFx.applied, floor: { session: 's' } } })
    expect((await getFloorState()).ok).toBe(false)
  })
  it('startSweep and stopSweep take the sweep state', async () => {
    const f = stub(200, stateFx.sweep)
    expect(await startSweep()).toEqual({ ok: true, value: stateFx.sweep })
    expect(f).toHaveBeenCalledWith(`${B}/sweep/start`, { method: 'POST' })
    expect(await stopSweep()).toEqual({ ok: true, value: stateFx.sweep })
    expect(f).toHaveBeenLastCalledWith(`${B}/sweep/stop`, { method: 'POST' })
  })
  it('computeFloor and getFloorResult accept the result fixture', async () => {
    const f = stub(200, resultFx)
    expect(await computeFloor('floor a/b')).toEqual({ ok: true, value: resultFx })
    expect(f).toHaveBeenCalledWith(`${B}/sessions/floor%20a%2Fb/compute`, { method: 'POST' })
    expect(await getFloorResult('s', 'r')).toEqual({ ok: true, value: resultFx })
    expect(f).toHaveBeenLastCalledWith(`${B}/sessions/s/results/r`, { cache: 'no-store' })
  })
  it('applyFloor accepts the apply fixture', async () => {
    const f = stub(200, applyFx)
    expect(await applyFloor('s', 'r')).toEqual({ ok: true, value: applyFx })
    expect(f).toHaveBeenCalledWith(`${B}/sessions/s/results/r/apply`, { method: 'POST' })
  })
  it('rejects a malformed body as an error', async () => {
    stub(200, { sweep: { running: 'yes' } })
    const r = await getFloorState()
    expect(r.ok).toBe(false)
    stub(200, { ...resultFx, params: { mount_z_m: { value: 'x' } } })
    expect((await getFloorResult('s', 'r')).ok).toBe(false)
    stub(200, { ...resultFx, passed: undefined })
    expect((await computeFloor('s')).ok).toBe(false)
    stub(200, { units: [] })
    expect((await applyFloor('s', 'r')).ok).toBe(false)
    stub(200, { running: true })
    expect((await startSweep()).ok).toBe(false)
  })
  it('accepts an older result without tilt roll and observation RMS, rejects a malformed new key', async () => {
    const { tilt_roll_deg: _t, ...params } = resultFx.params
    const { observation_height_rms_mm: _h, observation_normal_rms_deg: _n, ...old } = resultFx
    stub(200, { ...old, params })
    expect((await computeFloor('s')).ok).toBe(true)
    stub(200, { ...resultFx, params: { ...resultFx.params, tilt_roll_deg: { value: 'x' } } })
    expect((await computeFloor('s')).ok).toBe(false)
    stub(200, { ...resultFx, observation_height_rms_mm: 'x' })
    expect((await computeFloor('s')).ok).toBe(false)
  })
  it('error bodies give the node message', async () => {
    stub(409, { ok: false, message: 'a floor sweep is running; stop it first' })
    expect(await stopSweep()).toEqual({ ok: false, status: 409, message: 'a floor sweep is running; stop it first' })
  })
  it('network error is an error', async () => {
    vi.stubGlobal('fetch', vi.fn(async () => { throw new Error('link down') }))
    expect(await getFloorState()).toEqual({ ok: false, status: 0, message: 'link down' })
  })
})
