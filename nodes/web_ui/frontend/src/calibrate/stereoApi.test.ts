import { afterEach, describe, expect, it, vi } from 'vitest'
import {
  captureThumbUrl,
  capturableColor,
  deleteCaptures,
  fetchCaptures,
  fetchPreview,
  fetchSolve,
  isFinalSolveState,
  postCapture,
  postSolve,
  previewImageUrl,
} from './stereoApi'

function mockFetch(status: number, body: unknown) {
  const fn = vi.fn(async () => new Response(JSON.stringify(body), { status }))
  vi.stubGlobal('fetch', fn)
  return fn
}

afterEach(() => {
  vi.unstubAllGlobals()
})

describe('pure helpers', () => {
  it('capturable colour', () => {
    expect(capturableColor(true)).toBe('success.main')
    expect(capturableColor(false)).toBe('error.main')
  })
  it('seq matched urls', () => {
    expect(previewImageUrl('left', 7)).toBe('/api/calibration/stereo/preview/left.jpg?seq=7')
    expect(previewImageUrl('right', 7)).toBe('/api/calibration/stereo/preview/right.jpg?seq=7')
    expect(captureThumbUrl('c1', 'right')).toBe('/api/calibration/stereo/captures/c1/right.jpg')
  })
  it('final states', () => {
    for (const s of ['written', 'refused', 'failed', 'timeout']) expect(isFinalSolveState(s)).toBe(true)
    for (const s of ['idle', 'running']) expect(isFinalSolveState(s)).toBe(false)
  })
})

describe('fetch calls', () => {
  it('fetchPreview ok', async () => {
    const body = { seq: 3, left: { corners: 1 }, right: { corners: 2 }, capturable: true }
    const f = mockFetch(200, body)
    const r = await fetchPreview()
    expect(f).toHaveBeenCalledWith('/api/calibration/stereo/preview', { cache: 'no-store' })
    expect(r).toEqual({ ok: true, value: body })
  })
  it('fetchPreview rejects a malformed body', async () => {
    mockFetch(200, { hello: 1 })
    expect((await fetchPreview()).ok).toBe(false)
  })
  it('fetchPreview 503 gives the server reason', async () => {
    mockFetch(503, { reason: 'no fresh pair' })
    expect(await fetchPreview()).toEqual({ ok: false, status: 503, message: 'no fresh pair' })
  })
  it('proxy message used when no reason', async () => {
    mockFetch(502, { ok: false, message: 'Jetson depthd unreachable' })
    expect(await fetchPreview()).toEqual({ ok: false, status: 502, message: 'Jetson depthd unreachable' })
  })
  it('network error becomes a message', async () => {
    vi.stubGlobal('fetch', vi.fn(async () => { throw new Error('offline') }))
    expect(await fetchPreview()).toEqual({ ok: false, status: 0, message: 'offline' })
  })
  it('postCapture 201 and 422', async () => {
    const f = mockFetch(201, { id: 'a', captures: 4 })
    expect(await postCapture()).toEqual({ ok: true, value: { id: 'a', captures: 4 } })
    expect(f).toHaveBeenCalledWith('/api/calibration/stereo/captures', { method: 'POST' })
    mockFetch(422, { reason: 'only 3 shared corners, need 12' })
    expect(await postCapture()).toEqual({ ok: false, status: 422, message: 'only 3 shared corners, need 12' })
  })
  it('fetchCaptures', async () => {
    mockFetch(200, { captures: [], count: 0, min_pairs_to_solve: 10 })
    expect((await fetchCaptures()).ok).toBe(true)
  })
  it('deleteCaptures sends confirm=true', async () => {
    const f = mockFetch(200, { removed_captures: 2, calibration_removed: true })
    await deleteCaptures()
    expect(f).toHaveBeenCalledWith('/api/calibration/stereo/captures?confirm=true', { method: 'DELETE' })
  })
  it('postSolve and fetchSolve', async () => {
    const f = mockFetch(202, { state: 'running' })
    expect(await postSolve()).toEqual({ ok: true, value: { state: 'running' } })
    expect(f).toHaveBeenCalledWith('/api/calibration/stereo/solve', { method: 'POST' })
    const g = mockFetch(200, { state: 'written' })
    await fetchSolve()
    expect(g).toHaveBeenCalledWith('/api/calibration/stereo/solve', { cache: 'no-store' })
  })
})
