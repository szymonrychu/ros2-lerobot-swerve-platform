// @vitest-environment jsdom
import { afterEach, describe, expect, it, vi } from 'vitest'
import {
  applyResult, computeSession, deletePose, downloadUrl, emptyCells, getApplied, getResult, getSession, getState,
  listSessions, poseErrors, startCapture, statusSeverity, stopCapture, thumbnailUrl,
} from './rgbdApi'
import captureStartFx from '../../../../head_rgbd_calibration/tests/fixtures/api/capture_start.json'
import captureStopFx from '../../../../head_rgbd_calibration/tests/fixtures/api/capture_stop.json'
import applyFx from '../../../../head_rgbd_calibration/tests/fixtures/api/apply.json'
import appliedFx from '../../../../head_rgbd_calibration/tests/fixtures/api/applied.json'
import sessionFx from '../../../../head_rgbd_calibration/tests/fixtures/api/session.json'
import resultFx from '../../../../head_rgbd_calibration/tests/fixtures/api/result.json'
import stateFx from '../../../../head_rgbd_calibration/tests/fixtures/api/state.json'

const B = '/api/calibration/rgbd'
const state = {
  capture: { running: true, session_id: 's1', status: 'moving', pose_count: 2, started_at: '2026-01-01T00:00:00+00:00', stops_at: '2026-01-01T00:01:00+00:00' },
  compute: { running: false, session_id: null, result_id: null, started_at: null, error: null },
  applied: null,
}
const coverage = { cols: 2, rows: 2, counts: [[1, 0], [0, 3]] }
const check = { ...resultFx, kind: 'intrinsics' as const, id: 'r1', created: 'c' }
const detail = {
  id: 's1', created: 'c',
  poses: [{ id: 'p1', rgb_corners: 30, plane_bias_samples: 4200, hints: [] }],
  coverage: { rgb: coverage }, results: [check],
}
const result = { ...check, passed: false, report: '# rep' }
const applied = appliedFx

function stub(status: number, body: unknown) {
  const f = vi.fn(async () => new Response(JSON.stringify(body), { status }))
  vi.stubGlobal('fetch', f)
  return f
}
afterEach(() => {
  vi.unstubAllGlobals()
})

describe('rgbdApi helpers', () => {
  it('emptyCells counts zero cells over the colour grid', () => {
    expect(emptyCells(coverage)).toBe(2)
    expect(emptyCells({ cols: 1, rows: 1, counts: [[0]] })).toBe(1)
  })
  it('statusSeverity', () => {
    expect(statusSeverity('pose 3 captured - move to the next pose')).toBe('success')
    expect(statusSeverity('moving')).toBe('info')
    expect(statusSeverity('waiting for frames')).toBe('info')
    expect(statusSeverity('still - no board in the colour image')).toBe('warning')
    expect(statusSeverity('still - no colour camera_info yet')).toBe('warning')
  })
  it('urls encode ids and map kinds', () => {
    expect(thumbnailUrl('a/b', 'p 1', 'rgb')).toBe(`${B}/sessions/a%2Fb/poses/p%201/rgb.jpg`)
    expect(thumbnailUrl('s', 'p', 'depth')).toBe(`${B}/sessions/s/poses/p/depth.jpg`)
    expect(downloadUrl('s?', 'r#')).toBe(`${B}/sessions/s%3F/results/r%23/download`)
  })
})

describe('rgbdApi calls', () => {
  it('getState', async () => {
    const f = stub(200, state)
    expect(await getState()).toEqual({ ok: true, value: state })
    expect(f).toHaveBeenCalledWith(`${B}/state`, { cache: 'no-store' })
  })
  it('startCapture with and without session', async () => {
    const f = stub(200, state)
    await startCapture()
    expect(f).toHaveBeenCalledWith(`${B}/capture/start`, {
      method: 'POST', headers: { 'Content-Type': 'application/json' }, body: '{}',
    })
    await startCapture('s1')
    expect(f).toHaveBeenLastCalledWith(`${B}/capture/start`, {
      method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ session_id: 's1' }),
    })
  })
  it('stopCapture', async () => {
    const f = stub(200, state)
    await stopCapture()
    expect(f).toHaveBeenCalledWith(`${B}/capture/stop`, { method: 'POST' })
  })
  it('listSessions and getSession', async () => {
    const list = [{ id: 's1', created: 'c', pose_count: 1, result_count: 0 }]
    let f = stub(200, list)
    expect(await listSessions()).toEqual({ ok: true, value: list })
    expect(f).toHaveBeenCalledWith(`${B}/sessions`, { cache: 'no-store' })
    f = stub(200, detail)
    expect(await getSession('s/1')).toEqual({ ok: true, value: detail })
    expect(f).toHaveBeenCalledWith(`${B}/sessions/s%2F1`, { cache: 'no-store' })
  })
  it('deletePose', async () => {
    const f = stub(200, { ok: true })
    expect((await deletePose('s', 'p 1')).ok).toBe(true)
    expect(f).toHaveBeenCalledWith(`${B}/sessions/s/poses/p%201`, { method: 'DELETE' })
  })
  it('computeSession returns result_id', async () => {
    const f = stub(202, { result_id: 'r9' })
    expect(await computeSession('s')).toEqual({ ok: true, value: { result_id: 'r9' } })
    expect(f).toHaveBeenCalledWith(`${B}/sessions/s/compute`, { method: 'POST' })
  })
  it('getResult carries the report', async () => {
    const f = stub(200, result)
    expect(await getResult('s', 'r1')).toEqual({ ok: true, value: result })
    expect(f).toHaveBeenCalledWith(`${B}/sessions/s/results/r1`, { cache: 'no-store' })
  })
  it('getResult accepts a null report', async () => {
    const noReport = { ...result, report: null }
    stub(200, noReport)
    expect(await getResult('s', 'r1')).toEqual({ ok: true, value: noReport })
  })
  it('getState accepts an iso compute started_at', async () => {
    const st = { ...state, compute: { ...state.compute, running: true, started_at: '2026-01-01T00:00:00+00:00' } }
    stub(200, st)
    expect((await getState()).ok).toBe(true)
  })
  it('applyResult and getApplied', async () => {
    let f = stub(200, applied)
    expect(await applyResult('s', 'r1')).toEqual({ ok: true, value: applied })
    expect(f).toHaveBeenCalledWith(`${B}/sessions/s/results/r1/apply`, { method: 'POST' })
    f = stub(200, applied)
    expect((await getApplied()).ok).toBe(true)
    expect(f).toHaveBeenCalledWith(`${B}/applied`, { cache: 'no-store' })
  })
  it('server message passes through verbatim', async () => {
    stub(409, { ok: false, message: 'capture running' })
    expect(await computeSession('s')).toEqual({ ok: false, status: 409, message: 'capture running' })
    stub(404, { ok: false, message: 'nothing applied' })
    expect(await getApplied()).toEqual({ ok: false, status: 404, message: 'nothing applied' })
  })
  it('network failure and non-JSON error', async () => {
    vi.stubGlobal('fetch', vi.fn(async () => { throw new Error('down') }))
    expect(await getState()).toEqual({ ok: false, status: 0, message: 'down' })
    vi.stubGlobal('fetch', vi.fn(async () => new Response('x', { status: 502 })))
    expect(await getState()).toEqual({ ok: false, status: 502, message: 'HTTP 502' })
  })
  it('malformed 200 bodies are errors', async () => {
    stub(200, { capture: {} })
    expect((await getState()).ok).toBe(false)
    stub(200, [{ id: 1 }])
    expect((await listSessions()).ok).toBe(false)
    stub(200, { ...detail, coverage: { rgb: { cols: 1 } } })
    expect((await getSession('s')).ok).toBe(false)
    stub(200, { result_id: 3 })
    expect((await computeSession('s')).ok).toBe(false)
    stub(200, { id: 'r1', passed: true })
    expect((await getResult('s', 'r1')).ok).toBe(false)
    stub(200, { units: 'x' })
    expect((await getApplied()).ok).toBe(false)
    const { kind: _kind, ...withoutKind } = appliedFx
    void _kind
    stub(200, withoutKind)
    expect((await getApplied()).ok).toBe(false)
  })
})

describe('rgbdApi accepts the node contract fixtures', () => {
  it('getState accepts state.json', async () => {
    stub(200, stateFx)
    expect(await getState()).toEqual({ ok: true, value: stateFx })
  })
  it('startCapture accepts capture_start.json', async () => {
    stub(200, captureStartFx)
    expect(await startCapture()).toEqual({ ok: true, value: captureStartFx })
  })
  it('stopCapture accepts capture_stop.json', async () => {
    stub(200, captureStopFx)
    expect(await stopCapture()).toEqual({ ok: true, value: captureStopFx })
  })
  it('applyResult accepts apply.json and exposes the node key names', async () => {
    stub(200, applyFx)
    const r = await applyResult('s', 'r')
    expect(r).toEqual({ ok: true, value: applyFx })
    if (r.ok) {
      expect(r.value.session).toBe(applyFx.session)
      expect(r.value.result).toBe(applyFx.result)
      expect(r.value.units[0].name).toBe('ros2-head_camera_cpp.service')
    }
  })
  it('getApplied accepts applied.json', async () => {
    stub(200, appliedFx)
    expect(await getApplied()).toEqual({ ok: true, value: appliedFx })
  })
  it('a malformed start body still yields the unexpected-response error', async () => {
    stub(200, { capture: { running: 'yes' } })
    const r = await startCapture()
    expect(r.ok).toBe(false)
    if (!r.ok) expect(r.message).toContain('Unexpected calibration response')
  })
})

describe('rgbdApi pose hints and errors', () => {
  it('getSession accepts session.json with hints', async () => {
    stub(200, sessionFx)
    expect(await getSession('s')).toEqual({ ok: true, value: sessionFx })
  })
  it('getResult accepts result.json with the per-still errors', async () => {
    stub(200, resultFx)
    expect(await getResult('s', 'r')).toEqual({ ok: true, value: resultFx })
  })
  it('getResult exposes the intrinsics check fields of result.json', async () => {
    stub(200, resultFx)
    const r = await getResult('s', 'r')
    expect(r.ok).toBe(true)
    if (r.ok) {
      expect(r.value.kind).toBe('intrinsics')
      expect(r.value.rms_px).toBe(resultFx.rms_px)
      expect(r.value.focal_diff_pct).toBe(resultFx.focal_diff_pct)
      expect(r.value.principal_diff_px).toBe(resultFx.principal_diff_px)
      expect(r.value.plane_bias_median_mm).toBe(resultFx.plane_bias_median_mm)
      expect(r.value.plane_bias_spread_mm).toBe(resultFx.plane_bias_spread_mm)
      expect(r.value.failed_limits).toEqual([])
      expect(r.value.limits.intrinsics_max_rms_px).toBe(resultFx.limits.intrinsics_max_rms_px)
    }
  })
  it('accepts a failed input check with null metrics and an error', async () => {
    const failed = {
      ...resultFx, passed: false, error: 'calibration needs at least 12 views, got 3 views', rms_px: null,
      focal_diff_pct: null, principal_diff_px: null, plane_bias_median_mm: null, plane_bias_spread_mm: null, views: null, poses: [],
    }
    stub(200, failed)
    expect(await getResult('s', 'r')).toEqual({ ok: true, value: failed })
  })
  it('rejects a result that is not an intrinsics check', async () => {
    const { rms_px: _r, ...noRms } = resultFx
    void _r
    const { limits: _l, ...noLimits } = resultFx
    void _l
    const { failed_limits: _f, ...noFailed } = resultFx
    void _f
    const { poses: _p, ...noPoses } = resultFx
    void _p
    for (const bad of [
      { ...resultFx, kind: 'rgbd' },
      noRms,
      noLimits,
      noFailed,
      noPoses,
      { ...resultFx, rms_px: '0.2' },
      { ...resultFx, failed_limits: [1] },
      { ...resultFx, views: 'many' },
      { ...resultFx, limits: { ...resultFx.limits, min_views: 'x' } },
      { ...resultFx, error: 3 },
    ]) {
      stub(200, bad)
      expect((await getResult('s', 'r')).ok).toBe(false)
    }
  })
  it('rejects a pose without hints', async () => {
    stub(200, { ...detail, poses: [{ id: 'p1', rgb_corners: 30, plane_bias_samples: 4200 }] })
    expect((await getSession('s')).ok).toBe(false)
  })
  it('rejects a non-array hints / poses and wrong-typed entries', async () => {
    stub(200, { ...detail, poses: [{ ...detail.poses[0], hints: 'few' }] })
    expect((await getSession('s')).ok).toBe(false)
    stub(200, { ...detail, poses: [{ ...detail.poses[0], hints: [1] }] })
    expect((await getSession('s')).ok).toBe(false)
    stub(200, { ...detail, results: [{ ...check, poses: 'x' }] })
    expect((await getSession('s')).ok).toBe(false)
    stub(200, { ...result, poses: {} })
    expect((await getResult('s', 'r1')).ok).toBe(false)
    stub(200, { ...result, poses: [{ id: '1', error_px: 'a', plane_bias_mm: null, plane_spread_mm: null }] })
    expect((await getResult('s', 'r1')).ok).toBe(false)
    stub(200, { ...detail, poses: [{ id: 'p1', rgb_corners: 30 }] })
    expect((await getSession('s')).ok).toBe(false)
  })
})

describe('poseErrors', () => {
  const pe = { id: '001', error_px: 0.2, plane_bias_mm: -1.5, plane_spread_mm: null }
  it('is empty without results or without poses', () => {
    expect(poseErrors({ ...detail, results: [] }).size).toBe(0)
    expect(poseErrors({ ...detail, results: [{ ...check, poses: [] }] }).size).toBe(0)
  })
  it('keys the newest result poses by id', () => {
    const d = { ...detail, results: [{ ...check, id: 'new', passed: false, poses: [pe] }, { ...check, id: 'old', poses: [{ ...pe, id: '002' }] }] }
    const m = poseErrors(d)
    expect([...m.keys()]).toEqual(['001'])
    expect(m.get('001')).toEqual(pe)
  })
})
