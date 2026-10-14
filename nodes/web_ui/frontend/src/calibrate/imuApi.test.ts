// @vitest-environment jsdom
import { afterEach, describe, expect, it, vi } from 'vitest'
import { fetchImuStatus, imuLevelColor, postImuReset, postImuSave, saveEnabled } from './imuApi'

const fresh = { sys: 3, gyro: 3, accel: 2, mag: 3, restored: false, age_s: 0.4 }

function stub(status: number, body: unknown) {
  const f = vi.fn(async () => new Response(JSON.stringify(body), { status }))
  vi.stubGlobal('fetch', f)
  return f
}
afterEach(() => {
  vi.unstubAllGlobals()
})

describe('imuApi helpers', () => {
  it('level colour: 0-1 error, 2 warning, 3 success', () => {
    expect(imuLevelColor(0)).toBe('error.main')
    expect(imuLevelColor(1)).toBe('error.main')
    expect(imuLevelColor(2)).toBe('warning.main')
    expect(imuLevelColor(3)).toBe('success.main')
  })
  it('save enabled only when mag is 3', () => {
    expect(saveEnabled(fresh)).toBe(true)
    expect(saveEnabled({ ...fresh, mag: 2 })).toBe(false)
    expect(saveEnabled(null)).toBe(false)
  })
})

describe('imuApi calls', () => {
  it('fresh status', async () => {
    stub(200, fresh)
    expect(await fetchImuStatus()).toEqual({ ok: true, value: fresh })
  })
  it('stale and missing status carry the server message', async () => {
    stub(503, { ok: false, message: 'IMU calibration status stale (9 s)' })
    expect(await fetchImuStatus()).toEqual({ ok: false, status: 503, message: 'IMU calibration status stale (9 s)' })
    stub(503, { ok: false, message: 'no IMU calibration sample yet' })
    const r = await fetchImuStatus()
    expect(r.ok === false && r.message).toBe('no IMU calibration sample yet')
  })
  it('a malformed 200 is an error, never zeros', async () => {
    stub(200, { sys: 'x' })
    expect((await fetchImuStatus()).ok).toBe(false)
  })
  it('save success and refusal', async () => {
    const f = stub(200, { ok: true, message: 'saved' })
    expect(await postImuSave()).toEqual({ ok: true, value: { ok: true, message: 'saved' } })
    expect(f).toHaveBeenCalledWith('/api/calibration/imu/save', { method: 'POST' })
    stub(409, { ok: false, message: 'mag not calibrated' })
    expect(await postImuSave()).toEqual({ ok: false, status: 409, message: 'mag not calibrated' })
  })
  it('reset posts to reset', async () => {
    const f = stub(200, { ok: true, message: 'reset' })
    await postImuReset()
    expect(f).toHaveBeenCalledWith('/api/calibration/imu/reset', { method: 'POST' })
  })
})
