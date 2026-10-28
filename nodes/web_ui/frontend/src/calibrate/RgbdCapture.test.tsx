// @vitest-environment jsdom
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { act, cleanup, fireEvent, render, screen, waitFor } from '@testing-library/react'
import { RgbdCalibration } from './RgbdCalibration'
import { CoverageGrid } from './CoverageGrid'
import type { CaptureState, Coverage } from './rgbdApi'
import { RgbdCapture } from './RgbdCapture'
import captureStartFx from '../../../../head_rgbd_calibration/tests/fixtures/api/capture_start.json'
import captureStopFx from '../../../../head_rgbd_calibration/tests/fixtures/api/capture_stop.json'

const IDLE: CaptureState = { running: false, session_id: null, status: 'idle', pose_count: 0, started_at: null, stops_at: null }
const COMPUTE = { running: false, session_id: null, result_id: null, started_at: null, error: null }
const cov = (counts: number[][]): Coverage => ({ cols: counts[0].length, rows: counts.length, counts })

interface World {
  capture: typeof IDLE
  poses: { id: string; rgb_corners: number; plane_bias_samples: number }[]
  calls: string[]
}

function session(w: World) {
  return {
    id: 's1',
    created: '2026-10-19T10:00:00',
    poses: w.poses,
    coverage: { rgb: cov([[1, 0], [0, 0]]) },
    results: [],
  }
}

function install(w: World): void {
  vi.stubGlobal(
    'fetch',
    vi.fn(async (url: string, init?: RequestInit) => {
      const method = init?.method ?? 'GET'
      w.calls.push(`${method} ${url}`)
      const json = (b: unknown, status = 200) => new Response(JSON.stringify(b), { status })
      if (url.endsWith('/state')) return json({ capture: w.capture, compute: COMPUTE, applied: null })
      if (url.endsWith('/capture/start')) {
        w.capture = { ...w.capture, running: true, session_id: 's1', status: 'still - waiting', started_at: 'a', stops_at: 'b' }
        return json({ capture: w.capture, compute: COMPUTE, applied: null })
      }
      if (url.endsWith('/capture/stop')) {
        w.capture = { ...w.capture, running: false }
        return json({ capture: w.capture, compute: COMPUTE, applied: null })
      }
      if (method === 'DELETE') {
        w.poses = w.poses.filter((p) => !url.endsWith(`/poses/${p.id}`))
        return json({ ok: true })
      }
      if (url.endsWith('/sessions/s1')) return json(session(w))
      return json({ message: 'nope' }, 404)
    }),
  )
}

const pose = (id: string) => ({ id, rgb_corners: 12, plane_bias_samples: 4200 })
const stateCalls = (w: World) => w.calls.filter((c) => c.endsWith('/state')).length

describe('RgbdCapture', () => {
  beforeEach(() => {
    vi.useFakeTimers({ shouldAdvanceTime: true })
  })
  afterEach(() => {
    cleanup()
    vi.useRealTimers()
    vi.unstubAllGlobals()
  })

  it('a reload mid-capture resumes polling from the mounted state', async () => {
    const w: World = { capture: { ...IDLE, running: true, session_id: 's1', status: 'moving', pose_count: 1 }, poses: [pose('p1')], calls: [] }
    install(w)
    render(<RgbdCalibration />)
    await screen.findByText(/moving/)
    const before = stateCalls(w)
    await act(async () => { await vi.advanceTimersByTimeAsync(2100) })
    expect(stateCalls(w)).toBeGreaterThan(before)
    await screen.findByAltText('RGB p1')
  })

  it('start shows status and count, a new pose shows thumbnails', async () => {
    const w: World = { capture: IDLE, poses: [], calls: [] }
    install(w)
    render(<RgbdCalibration />)
    fireEvent.click(await screen.findByRole('button', { name: 'Start capture' }))
    await screen.findByText(/still - waiting/)
    expect(screen.getByText(/Poses: 0/)).toBeTruthy()
    w.capture = { ...w.capture, status: 'pose 1 captured', pose_count: 1 }
    w.poses = [pose('p1')]
    await act(async () => { await vi.advanceTimersByTimeAsync(1100) })
    await screen.findByText(/pose 1 captured/)
    expect(screen.getByText(/Poses: 1/)).toBeTruthy()
    const rgb = await screen.findByAltText('RGB p1')
    expect(rgb.getAttribute('src')).toBe('/api/calibration/rgbd/sessions/s1/poses/p1/rgb.jpg')
    expect(screen.getByAltText('Depth p1').getAttribute('src')).toBe('/api/calibration/rgbd/sessions/s1/poses/p1/depth.jpg')
    expect(screen.queryByAltText('ToF p1')).toBeNull()
    expect(screen.getByText('RGB 12 corners, 4200 depth px on the board')).toBeTruthy()
  })

  it('delete goes through the dialog, calls DELETE and refreshes', async () => {
    const w: World = { capture: IDLE, poses: [pose('p1'), pose('p2')], calls: [] }
    install(w)
    w.capture = { ...IDLE, running: true, session_id: 's1', status: 'moving', pose_count: 2 }
    render(<RgbdCalibration />)
    await screen.findByAltText('RGB p2')
    fireEvent.click(screen.getByRole('button', { name: 'Delete pose p2' }))
    expect(w.calls.some((c) => c.startsWith('DELETE'))).toBe(false)
    fireEvent.click(await screen.findByRole('button', { name: 'Delete' }))
    await waitFor(() => expect(w.calls).toContain('DELETE /api/calibration/rgbd/sessions/s1/poses/p2'))
    await waitFor(() => expect(screen.queryByAltText('RGB p2')).toBeNull())
    expect(screen.getByAltText('RGB p1')).toBeTruthy()
  })

  it('delete failure shows the server message', async () => {
    const w: World = { capture: { ...IDLE, running: true, session_id: 's1', status: 'moving', pose_count: 1 }, poses: [pose('p1')], calls: [] }
    install(w)
    const base = fetch as unknown as (u: string, i?: RequestInit) => Promise<Response>
    vi.stubGlobal('fetch', vi.fn(async (u: string, i?: RequestInit) =>
      i?.method === 'DELETE' ? new Response(JSON.stringify({ message: 'pose is busy' }), { status: 409 }) : base(u, i)))
    render(<RgbdCalibration />)
    await screen.findByAltText('RGB p1')
    fireEvent.click(screen.getByRole('button', { name: 'Delete pose p1' }))
    fireEvent.click(await screen.findByRole('button', { name: 'Delete' }))
    await screen.findByText(/pose is busy/)
  })

  it('polling stops on stop and pauses while hidden', async () => {
    const w: World = { capture: IDLE, poses: [], calls: [] }
    install(w)
    render(<RgbdCalibration />)
    fireEvent.click(await screen.findByRole('button', { name: 'Start capture' }))
    await screen.findByText(/still - waiting/)

    const vis = vi.spyOn(document, 'visibilityState', 'get').mockReturnValue('hidden')
    document.dispatchEvent(new Event('visibilitychange'))
    let n = stateCalls(w)
    await act(async () => { await vi.advanceTimersByTimeAsync(3100) })
    expect(stateCalls(w)).toBe(n)
    vis.mockReturnValue('visible')
    document.dispatchEvent(new Event('visibilitychange'))
    await act(async () => { await vi.advanceTimersByTimeAsync(1100) })
    expect(stateCalls(w)).toBeGreaterThan(n)

    fireEvent.click(screen.getByRole('button', { name: 'Stop' }))
    await waitFor(() => expect(w.calls).toContain('POST /api/calibration/rgbd/capture/stop'))
    await act(async () => { await vi.advanceTimersByTimeAsync(100) })
    n = stateCalls(w)
    await act(async () => { await vi.advanceTimersByTimeAsync(3100) })
    expect(stateCalls(w)).toBe(n)
  })

  it('start error shows the server message', async () => {
    vi.stubGlobal('fetch', vi.fn(async (u: string) =>
      u.endsWith('/state')
        ? new Response(JSON.stringify({ capture: IDLE, compute: COMPUTE, applied: null }))
        : new Response(JSON.stringify({ message: 'compute is running' }), { status: 409 })))
    render(<RgbdCalibration />)
    fireEvent.click(await screen.findByRole('button', { name: 'Start capture' }))
    await screen.findByText(/compute is running/)
  })
})

describe('CoverageGrid', () => {
  afterEach(cleanup)
  it('shows the empty-area hint for the colour image', () => {
    render(<CoverageGrid rgb={cov([[1, 0], [0, 0]])} />)
    expect(screen.getByText('3 of 4 image areas without corners')).toBeTruthy()
    expect(screen.getAllByTestId('coverage-cell-empty').length).toBe(3)
  })
})

describe('RgbdCapture with the node contract fixtures', () => {
  afterEach(() => {
    cleanup()
    vi.unstubAllGlobals()
  })

  const serve = (body: unknown) => vi.stubGlobal('fetch', vi.fn(async () => new Response(JSON.stringify(body), { status: 200 })))

  it('Start with capture_start.json hands the state up and shows no error', async () => {
    serve(captureStartFx)
    const onState = vi.fn()
    render(<RgbdCapture capture={IDLE} compute={COMPUTE} sessionId={null} onState={onState} />)
    fireEvent.click(screen.getByRole('button', { name: 'Start capture' }))
    await waitFor(() => expect(onState).toHaveBeenCalledWith(captureStartFx))
    expect(screen.queryByText(/Unexpected calibration response/)).toBeNull()
  })

  it('Stop with capture_stop.json hands the state up and shows no error', async () => {
    serve(captureStopFx)
    const onState = vi.fn()
    const running = { ...IDLE, ...captureStartFx.capture }
    render(<RgbdCapture capture={running} compute={COMPUTE} sessionId={null} onState={onState} />)
    fireEvent.click(screen.getByRole('button', { name: 'Stop' }))
    await waitFor(() => expect(onState).toHaveBeenCalledWith(captureStopFx))
    expect(screen.queryByText(/Unexpected calibration response/)).toBeNull()
  })

  it('RgbdCalibration shows the started capture from the Start answer', async () => {
    const sid = captureStartFx.capture.session_id
    const detail = { id: sid, created: 'c', poses: [], coverage: { rgb: cov([[0]]) }, results: [] }
    const f = vi.fn(async (url: string) => {
      const body = url.endsWith('/capture/start')
        ? captureStartFx
        : url.endsWith('/state')
          ? { ...captureStartFx, capture: IDLE }
          : url.endsWith('/sessions')
            ? []
            : detail
      return new Response(JSON.stringify(body), { status: 200 })
    })
    vi.stubGlobal('fetch', f)
    render(<RgbdCalibration />)
    fireEvent.click(await screen.findByRole('button', { name: 'Start capture' }))
    await screen.findByText(/waiting for frames/)
    expect(screen.queryByText(/Unexpected calibration response/)).toBeNull()
    expect(document.querySelector('.MuiAlert-colorError')?.textContent ?? null).toBeNull()
  })
})

describe('RgbdCapture pose hints and errors', () => {
  afterEach(() => {
    cleanup()
    vi.unstubAllGlobals()
  })
  const pe = (id: string) => ({ id, error_px: 0.21, plane_bias_mm: -1.5, plane_spread_mm: 2.0 })
  const detailWith = (results: unknown[]) => ({
    id: 's1',
    created: 'c',
    poses: [
      { ...pose('p1'), hints: ['few corners: 6 RGB - move the board closer', 'second hint'] },
      pose('p2'),
      pose('p3'),
    ],
    coverage: { rgb: cov([[1]]) },
    results,
  })
  const serve = (getBody: () => unknown) => {
    const f = vi.fn(async () => new Response(JSON.stringify(getBody()), { status: 200 }))
    vi.stubGlobal('fetch', f)
    return f
  }
  const mount = (compute = COMPUTE) =>
    render(<RgbdCapture capture={IDLE} compute={compute} sessionId="s1" onState={vi.fn()} />)

  it('shows each hint as its own line', async () => {
    serve(() => detailWith([]))
    mount()
    await screen.findByText('few corners: 6 RGB - move the board closer')
    expect(screen.getByText('second hint')).toBeTruthy()
    expect(screen.queryByText(/RGB 0\.210/)).toBeNull()
  })

  it('shows the error caption and skips poses absent from the result', async () => {
    serve(() => detailWith([{ id: 'r2', passed: false, poses: [pe('p1'), { ...pe('p2'), plane_bias_mm: null }] }]))
    mount()
    expect(await screen.findByText('RGB 0.210 px / plane bias -1.5 mm')).toBeTruthy()
    expect(screen.getByText('RGB 0.210 px / plane bias - mm')).toBeTruthy()
    expect(screen.getByTestId('pose-card-p3').textContent).not.toMatch(/plane bias/)
    expect(screen.getByRole('button', { name: 'Delete pose p2' })).toBeTruthy()
  })

  it('fetches the session again when a compute finishes', async () => {
    let results: unknown[] = []
    const f = serve(() => detailWith(results))
    const { rerender } = mount({ ...COMPUTE, running: true })
    await screen.findByAltText('RGB p2')
    expect(screen.queryByText(/plane bias/)).toBeNull()
    const before = f.mock.calls.length
    results = [{ id: 'r2', passed: false, poses: [pe('p2')] }]
    rerender(<RgbdCapture capture={IDLE} compute={COMPUTE} sessionId="s1" onState={vi.fn()} />)
    await screen.findByText('RGB 0.210 px / plane bias -1.5 mm')
    expect(f.mock.calls.length).toBeGreaterThan(before)
  })
})
