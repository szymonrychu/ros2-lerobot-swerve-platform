// @vitest-environment jsdom
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { act, cleanup, fireEvent, render, screen, waitFor } from '@testing-library/react'
import { RgbdCalibration } from './RgbdCalibration'
import { CoverageGrid } from './CoverageGrid'
import type { CaptureState, Coverage } from './rgbdApi'

const IDLE: CaptureState = { running: false, session_id: null, status: 'idle', pose_count: 0, started_at: null, stops_at: null }
const COMPUTE = { running: false, session_id: null, result_id: null, started_at: null, error: null }
const cov = (counts: number[][]): Coverage => ({ cols: counts[0].length, rows: counts.length, counts })

interface World {
  capture: typeof IDLE
  poses: { id: string; rgb_corners: number; tof_corners: number; has_depth: boolean }[]
  calls: string[]
}

function session(w: World) {
  return {
    id: 's1',
    created: '2026-10-19T10:00:00',
    lens_position: null,
    poses: w.poses,
    coverage: { rgb: cov([[1, 0], [0, 0]]), tof: cov([[1, 1], [0, 0]]) },
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

const pose = (id: string) => ({ id, rgb_corners: 12, tof_corners: 9, has_depth: true })
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
    expect(screen.getByAltText('ToF p1').getAttribute('src')).toContain('/poses/p1/tof.jpg')
    expect(screen.getByText(/RGB 12 corners/)).toBeTruthy()
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
  it('shows the empty-area hint for RGB and ToF', () => {
    render(<CoverageGrid rgb={cov([[1, 0], [0, 0]])} tof={cov([[1, 1], [0, 0]])} />)
    expect(screen.getByText('3 of 4 image areas without corners')).toBeTruthy()
    expect(screen.getByText('2 of 4 image areas without corners')).toBeTruthy()
    expect(screen.getAllByTestId('coverage-cell-empty').length).toBe(5)
  })
})
