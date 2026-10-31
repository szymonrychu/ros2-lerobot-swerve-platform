// @vitest-environment jsdom
import { afterEach, describe, expect, it, vi } from 'vitest'
import {
  CSI_CAMERAS,
  applyCsi,
  computeCsi,
  getCsiResult,
  getCsiState,
  listCsiResults,
  startCsiCapture,
  stopCsiCapture,
  type CsiApplyAnswer,
  type CsiResult,
  type CsiState,
} from './csiApi'

const B = '/api/calibration/rgbd/csi'

const RESULT: CsiResult = {
  kind: 'csi_intrinsics',
  camera: 'front',
  session: 'csi-20261031-120000',
  result: 'r-20261031-120500',
  created: '2026-10-31T12:05:00+00:00',
  image_size: [1152, 648],
  k: [
    [905.2, 0, 577.1],
    [0, 904.8, 322.6],
    [0, 0, 1],
  ],
  d: [0.04, -0.08, 0.0, 0.0, 0.0],
  rms: 0.312,
  per_view_errors: [0.3, 0.31],
  view_count: 14,
}

const STATE: CsiState = {
  camera: 'front',
  capture: {
    running: true,
    camera: 'front',
    session_id: 'csi-20261031-120000',
    status: 'view 14 kept - move the board',
    view_count: 14,
    started_at: '2026-10-31T12:00:00+00:00',
    stops_at: '2026-10-31T12:15:00+00:00',
  },
  session: {
    id: 'csi-20261031-120000',
    created: '2026-10-31T12:00:00+00:00',
    view_count: 14,
    image_size: [1152, 648],
    coverage: { cols: 4, rows: 3, counts: [[3, 0, 2, 1], [4, 5, 0, 2], [1, 1, 1, 0]] },
    results: [RESULT],
  },
  applied: null,
}

const APPLY: CsiApplyAnswer = {
  applied: {
    kind: 'csi_intrinsics',
    camera: 'front',
    session: 'csi-20261031-120000',
    result: 'r-20261031-120500',
    applied_at: '2026-10-31T12:06:00+00:00',
  },
  units: [{ name: 'ros2-front_camera.service', active_state: 'active', restarted: true }],
}

function stub(status: number, body: unknown) {
  const f = vi.fn(async () => new Response(JSON.stringify(body), { status }))
  vi.stubGlobal('fetch', f)
  return f
}

afterEach(() => {
  vi.unstubAllGlobals()
})

describe('csiApi', () => {
  it('names the front and back cameras', () => {
    expect(CSI_CAMERAS).toEqual(['front', 'back'])
  })
  it('getCsiState hits the camera state route', async () => {
    const f = stub(200, STATE)
    expect(await getCsiState('front')).toEqual({ ok: true, value: STATE })
    expect(f).toHaveBeenCalledWith(`${B}/front/state`, { cache: 'no-store' })
  })
  it('accepts a state without a session and with an applied record', async () => {
    stub(200, { ...STATE, session: null, applied: APPLY.applied })
    expect((await getCsiState('back')).ok).toBe(true)
  })
  it('start posts the optional session id, stop posts', async () => {
    const f = stub(200, STATE)
    expect((await startCsiCapture('back')).ok).toBe(true)
    expect(f).toHaveBeenCalledWith(`${B}/back/start`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: '{}',
    })
    await startCsiCapture('front', 'csi-1')
    expect(f).toHaveBeenLastCalledWith(`${B}/front/start`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ session_id: 'csi-1' }),
    })
    expect((await stopCsiCapture('front')).ok).toBe(true)
    expect(f).toHaveBeenLastCalledWith(`${B}/front/stop`, { method: 'POST' })
  })
  it('compute, results and result hit the session routes', async () => {
    const f = stub(200, RESULT)
    expect(await computeCsi('front', 'csi a/b')).toEqual({ ok: true, value: RESULT })
    expect(f).toHaveBeenCalledWith(`${B}/front/sessions/csi%20a%2Fb/compute`, { method: 'POST' })
    expect(await getCsiResult('front', 's', 'r')).toEqual({ ok: true, value: RESULT })
    expect(f).toHaveBeenLastCalledWith(`${B}/front/sessions/s/results/r`, { cache: 'no-store' })
    stub(200, [RESULT])
    expect(await listCsiResults('back', 's')).toEqual({ ok: true, value: [RESULT] })
    expect(fetch).toHaveBeenCalledWith(`${B}/back/sessions/s/results`, { cache: 'no-store' })
  })
  it('apply posts and takes the record and the units', async () => {
    const f = stub(200, APPLY)
    expect(await applyCsi('front', 's', 'r')).toEqual({ ok: true, value: APPLY })
    expect(f).toHaveBeenCalledWith(`${B}/front/sessions/s/results/r/apply`, { method: 'POST' })
  })
  it('rejects malformed bodies', async () => {
    stub(200, { ...STATE, capture: { running: 'yes' } })
    expect((await getCsiState('front')).ok).toBe(false)
    stub(200, { ...RESULT, k: [1, 2, 3] })
    expect((await computeCsi('front', 's')).ok).toBe(false)
    stub(200, { applied: APPLY.applied })
    expect((await applyCsi('front', 's', 'r')).ok).toBe(false)
  })
  it('error bodies give the node message', async () => {
    stub(409, { ok: false, message: 'calibration needs at least 12 views, got 3 views' })
    expect(await computeCsi('front', 's')).toEqual({
      ok: false,
      status: 409,
      message: 'calibration needs at least 12 views, got 3 views',
    })
  })
  it('network error is an error', async () => {
    vi.stubGlobal('fetch', vi.fn(async () => { throw new Error('link down') }))
    expect(await getCsiState('front')).toEqual({ ok: false, status: 0, message: 'link down' })
  })
})
