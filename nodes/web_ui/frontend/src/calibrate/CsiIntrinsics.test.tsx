// @vitest-environment jsdom
import { afterEach, describe, expect, it, vi } from 'vitest'
import { act, cleanup, fireEvent, render, screen, within } from '@testing-library/react'
import { CSI_POLL_MS, CsiIntrinsics } from './CsiIntrinsics'
import type { CsiCapture, CsiResult, CsiSession, CsiState } from './csiApi'

const B = '/api/calibration/rgbd/csi'
const SID = 'csi-20261031-120000'
const RID = 'r-20261031-120500'
const IDLE: CsiCapture = {
  running: false,
  camera: null,
  session_id: null,
  status: 'idle',
  view_count: 0,
  started_at: null,
  stops_at: null,
}
const RESULT: CsiResult = {
  kind: 'csi_intrinsics',
  camera: 'front',
  session: SID,
  result: RID,
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

function session(views: number): CsiSession {
  return {
    id: SID,
    created: '2026-10-31T12:00:00+00:00',
    view_count: views,
    image_size: [1152, 648],
    coverage: { cols: 4, rows: 3, counts: [[3, 0, 2, 1], [4, 5, 0, 2], [1, 1, 1, 0]] },
    results: [],
  }
}

interface World {
  capture: CsiCapture
  session: CsiSession | null
  calls: string[]
  compute: { status: number; body: unknown }
}

function install(w: World): void {
  vi.stubGlobal(
    'fetch',
    vi.fn(async (url: string, init?: RequestInit) => {
      w.calls.push(`${init?.method ?? 'GET'} ${url}`)
      const json = (b: unknown, status = 200) => new Response(JSON.stringify(b), { status })
      const camera = url.slice(B.length + 1).split('/')[0]
      const state = (): CsiState => ({ camera, capture: w.capture, session: w.session, applied: null })
      if (url.endsWith('/state')) return json(state())
      if (url.endsWith('/start')) {
        w.capture = { ...IDLE, running: true, camera, session_id: SID, status: 'waiting for frames', started_at: 'a', stops_at: 'b' }
        w.session = session(0)
        return json(state())
      }
      if (url.endsWith('/stop')) {
        w.capture = { ...w.capture, running: false, status: 'stopped' }
        return json(state())
      }
      if (url.endsWith('/compute')) return json(w.compute.body, w.compute.status)
      if (url.endsWith('/apply')) {
        return json({
          applied: { kind: 'csi_intrinsics', camera, session: SID, result: RID, applied_at: '2026-10-31T12:06:00+00:00' },
          units: [{ name: `ros2-${camera}_camera.service`, active_state: 'active', restarted: true }],
        })
      }
      return json({ ok: false, message: 'nope' }, 404)
    }),
  )
}

const world = (over: Partial<World> = {}): World => ({
  capture: IDLE,
  session: null,
  calls: [],
  compute: { status: 200, body: RESULT },
  ...over,
})

afterEach(() => {
  cleanup()
  vi.unstubAllGlobals()
  vi.useRealTimers()
})

describe('CsiIntrinsics', () => {
  it('shows the section with a front and back camera selector, front first', async () => {
    const w = world()
    install(w)
    render(<CsiIntrinsics />)
    await screen.findByText('CSI camera intrinsics')
    const front = screen.getByRole('button', { name: 'Front' })
    const back = screen.getByRole('button', { name: 'Back' })
    expect(front.getAttribute('aria-pressed')).toBe('true')
    expect(back.getAttribute('aria-pressed')).toBe('false')
    expect(w.calls).toContain(`GET ${B}/front/state`)
    await act(async () => {
      fireEvent.click(back)
    })
    expect(w.calls).toContain(`GET ${B}/back/state`)
    expect(screen.getByRole('button', { name: 'Back' }).getAttribute('aria-pressed')).toBe('true')
  })

  it('start captures, polls the kept-view count with the coverage grid, stop ends it', async () => {
    vi.useFakeTimers()
    const w = world()
    install(w)
    render(<CsiIntrinsics />)
    await act(async () => {})
    await act(async () => {
      fireEvent.click(screen.getByRole('button', { name: 'Start capture' }))
    })
    expect(w.calls).toContain(`POST ${B}/front/start`)
    expect(screen.getByText(/0 views kept - waiting for frames/)).toBeTruthy()
    w.capture = { ...w.capture, view_count: 9, status: 'view 9 kept - move the board' }
    w.session = session(9)
    const before = w.calls.filter((c) => c.endsWith('/state')).length
    await act(async () => {
      await vi.advanceTimersByTimeAsync(CSI_POLL_MS * 2)
    })
    expect(w.calls.filter((c) => c.endsWith('/state')).length).toBeGreaterThan(before)
    expect(screen.getByText(/9 views kept - view 9 kept - move the board/)).toBeTruthy()
    expect(screen.getAllByTestId('coverage-cell-empty')).toHaveLength(3)
    expect(screen.getAllByTestId('coverage-cell')).toHaveLength(9)
    await act(async () => {
      fireEvent.click(screen.getByRole('button', { name: 'Stop' }))
    })
    expect(w.calls).toContain(`POST ${B}/front/stop`)
    expect(screen.getByRole('button', { name: 'Compute' }).hasAttribute('disabled')).toBe(false)
  })

  it('compute shows the RMS and the view count, apply posts and shows the restarted unit', async () => {
    const w = world({ session: session(14), capture: { ...IDLE, camera: 'front', session_id: SID, status: 'stopped', view_count: 14 } })
    install(w)
    render(<CsiIntrinsics />)
    const compute = await screen.findByRole('button', { name: 'Compute' })
    await act(async () => {
      fireEvent.click(compute)
    })
    expect(w.calls).toContain(`POST ${B}/front/sessions/${SID}/compute`)
    expect(screen.getByText(/RMS 0.312 px over 14 views/)).toBeTruthy()
    const fx = within(screen.getByRole('row', { name: /fx/ }))
    expect(fx.getByText('905.2')).toBeTruthy()
    await act(async () => {
      fireEvent.click(screen.getByRole('button', { name: 'Apply' }))
    })
    expect(w.calls).toContain(`POST ${B}/front/sessions/${SID}/results/${RID}/apply`)
    expect(screen.getByText(/ros2-front_camera.service: restarted \(active\)/)).toBeTruthy()
  })

  it('too few views shows the node message and no result', async () => {
    const message = 'calibration needs at least 12 views, got 3 views'
    const w = world({ session: session(3), compute: { status: 409, body: { ok: false, message } } })
    install(w)
    render(<CsiIntrinsics />)
    const compute = await screen.findByRole('button', { name: 'Compute' })
    await act(async () => {
      fireEvent.click(compute)
    })
    expect(screen.getByText(message)).toBeTruthy()
    expect(screen.queryByRole('button', { name: 'Apply' })).toBeNull()
  })

  it('shows the applied record of the camera', async () => {
    install(world())
    vi.stubGlobal(
      'fetch',
      vi.fn(async () =>
        new Response(
          JSON.stringify({
            camera: 'front',
            capture: IDLE,
            session: null,
            applied: { kind: 'csi_intrinsics', camera: 'front', session: SID, result: RID, applied_at: '2026-10-31T12:06:00+00:00' },
          }),
        ),
      ),
    )
    render(<CsiIntrinsics />)
    await screen.findByText(new RegExp(`Applied: session ${SID}, result ${RID}`))
  })
})
