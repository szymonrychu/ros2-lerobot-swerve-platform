// @vitest-environment jsdom
import { afterEach, describe, expect, it, vi } from 'vitest'
import {
  applyResult, computeSession, deletePose, downloadUrl, emptyCells, getApplied, getResult, getSession, getState,
  listSessions, startCapture, statusSeverity, stopCapture, thumbnailUrl,
} from './rgbdApi'

const B = '/api/calibration/rgbd'
const state = {
  capture: { running: true, session_id: 's1', status: 'moving', pose_count: 2, started_at: 1, stops_at: 9 },
  compute: { running: false, session_id: null, result_id: null, started_at: null, error: null },
  applied: null,
}
const coverage = { cols: 2, rows: 2, counts: [[1, 0], [0, 3]] }
const detail = {
  id: 's1', created: 'c', lens_position: 1.5,
  poses: [{ id: 'p1', rgb_corners: 30, tof_corners: 28, has_depth: true }],
  coverage: { rgb: coverage, tof: coverage }, results: [{ id: 'r1', created: 'c', passed: true }],
}
const result = { id: 'r1', created: 'c', passed: false, report: '# rep' }
const applied = { session_id: 's1', result_id: 'r1', applied_at: 'a', units: [{ name: 'u', restarted: true }] }

function stub(status: number, body: unknown) {
  const f = vi.fn(async () => new Response(JSON.stringify(body), { status }))
  vi.stubGlobal('fetch', f)
  return f
}
afterEach(() => {
  vi.unstubAllGlobals()
})

describe('rgbdApi helpers', () => {
  it('emptyCells counts zero cells over rgb or tof grid', () => {
    expect(emptyCells(coverage)).toBe(2)
    expect(emptyCells({ cols: 1, rows: 1, counts: [[0]] })).toBe(1)
  })
  it('statusSeverity', () => {
    expect(statusSeverity('pose 3 captured - move to the next pose')).toBe('success')
    expect(statusSeverity('moving')).toBe('info')
    expect(statusSeverity('waiting for frames')).toBe('info')
    expect(statusSeverity('still - no board in the RGB image')).toBe('warning')
    expect(statusSeverity('still - board not found by the ToF: move closer')).toBe('warning')
    expect(statusSeverity('still - no ToF frame close in time')).toBe('warning')
  })
  it('urls encode ids and map kinds', () => {
    expect(thumbnailUrl('a/b', 'p 1', 'rgb')).toBe(`${B}/sessions/a%2Fb/poses/p%201/rgb.jpg`)
    expect(thumbnailUrl('s', 'p', 'tof')).toBe(`${B}/sessions/s/poses/p/tof.jpg`)
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
    stub(200, { ...detail, coverage: { rgb: { cols: 1 }, tof: coverage } })
    expect((await getSession('s')).ok).toBe(false)
    stub(200, { result_id: 3 })
    expect((await computeSession('s')).ok).toBe(false)
    stub(200, { id: 'r1', passed: true })
    expect((await getResult('s', 'r1')).ok).toBe(false)
    stub(200, { units: 'x' })
    expect((await getApplied()).ok).toBe(false)
  })
})
