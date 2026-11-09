// @vitest-environment jsdom
import { afterEach, describe, expect, it, vi } from 'vitest'
import { GENERAL_ACTIONS, abortCalibration, runGeneralAction } from './generalApi'

const SEQ_IDLE = {
  running: false,
  phase: 'idle',
  session_id: null,
  tier: null,
  arm_pose: null,
  total_poses: 0,
  captured: 0,
  skipped: {},
  status: 'idle',
  error: null,
  big_joints: [],
  waiting_since: null,
  board_record: null,
  events: [],
}
const SWEEP_IDLE = { running: false, session_id: null, pose: 0, total: 0, observations: 0, skipped: 0, status: 'idle', error: null }
const CAPTURE_IDLE = { running: false, session_id: null, status: 'idle', pose_count: 0, started_at: null, stops_at: null }
const COMPUTE_IDLE = { running: false, session_id: null, result_id: null, started_at: null, error: null }
const BOARD = { name: 'b', dictionary: 'DICT_4X4_50', role: 'full', squares_x: 6, squares_y: 5, square_mm: 40, marker_mm: 30, ids_offset: 0, thickness_mm: 3 }
const CSI_IDLE = { running: false, camera: null, session_id: null, status: 'idle', view_count: 0, started_at: null, stops_at: null }

interface Running {
  headArm?: Partial<typeof SEQ_IDLE>
  sweep?: boolean
  d435?: boolean
  csi?: string | null
}

/** Fake node: states per `running`, every POST answered with the idle state of its family; records the POST urls. */
function fakeNode(running: Running, failing: string[] = []): string[] {
  const posts: string[] = []
  vi.stubGlobal(
    'fetch',
    vi.fn(async (url: string, init?: RequestInit) => {
      if (failing.some((f) => url.includes(f))) return new Response(JSON.stringify({ message: `down: ${url}` }), { status: 503 })
      if (init?.method === 'POST') posts.push(url)
      if (url.includes('/head-arm/')) {
        const sequence = { ...SEQ_IDLE, ...(init?.method === 'POST' ? {} : running.headArm) }
        return new Response(JSON.stringify(url.endsWith('/state') ? { sequence, applied: null } : sequence))
      }
      if (url.includes('/floor/')) {
        const sweep = { ...SWEEP_IDLE, running: init?.method !== 'POST' && running.sweep === true }
        return new Response(JSON.stringify(url.endsWith('/state') ? { sweep, applied: null } : sweep))
      }
      if (url.includes('/csi/')) {
        const capture = { ...CSI_IDLE, running: init?.method !== 'POST' && !!running.csi, camera: running.csi ?? null }
        return new Response(JSON.stringify({ camera: 'front', capture, session: null, applied: null }))
      }
      const capture = { ...CAPTURE_IDLE, running: init?.method !== 'POST' && running.d435 === true }
      return new Response(JSON.stringify({ capture, compute: COMPUTE_IDLE, applied: null, boards: { full: BOARD, arm: BOARD } }))
    }),
  )
  return posts
}

afterEach(() => {
  vi.unstubAllGlobals()
})

describe('runGeneralAction', () => {
  it('posts the action and returns the backend answer', async () => {
    const fetchMock = vi.fn(async () => new Response(JSON.stringify({ ok: true, message: 'converged: home' })))
    vi.stubGlobal('fetch', fetchMock)
    const r = await runGeneralAction('home_arm')
    expect(fetchMock).toHaveBeenCalledWith('/api/calibration/general/home_arm', { method: 'POST' })
    expect(r).toEqual({ ok: true, value: { ok: true, message: 'converged: home' } })
  })

  it('a refusal (409) returns the backend message', async () => {
    vi.stubGlobal('fetch', vi.fn(async () => new Response(JSON.stringify({ ok: false, message: 'Arm servo torque is off' }), { status: 409 })))
    expect(await runGeneralAction('open_gripper')).toEqual({ ok: false, status: 409, message: 'Arm servo torque is off' })
  })

  it('lists the five actions in the page order with a confirm text each', () => {
    expect(GENERAL_ACTIONS.map((a) => a.id)).toEqual(['open_gripper', 'close_gripper', 'home_arm', 'home_head', 'home_steering'])
    for (const a of GENERAL_ACTIONS) expect(a.confirm.length).toBeGreaterThan(10)
  })
})

describe('abortCalibration', () => {
  it('nothing running stops nothing', async () => {
    const posts = fakeNode({})
    const r = await abortCalibration()
    expect(posts).toEqual([])
    expect(r).toEqual({ stopped: [], errors: [] })
  })

  it('stops a running head-arm sequence (it holds the arm, never parks)', async () => {
    const posts = fakeNode({ headArm: { running: true, phase: 'tier' } })
    const r = await abortCalibration()
    expect(posts).toEqual(['/api/calibration/rgbd/head-arm/sequence/stop'])
    expect(posts.some((p) => p.includes('park'))).toBe(false)
    expect(r.stopped).toEqual(['arm-camera capture'])
  })

  it('stops a sequence waiting for the board or the big-joint consent', async () => {
    for (const phase of ['waiting_board', 'waiting_big_joints']) {
      const posts = fakeNode({ headArm: { running: false, phase } })
      await abortCalibration()
      expect(posts).toEqual(['/api/calibration/rgbd/head-arm/sequence/stop'])
    }
  })

  it('a park in progress is not stopped (the node never interrupts a park)', async () => {
    const posts = fakeNode({ headArm: { running: true, phase: 'parking' } })
    const r = await abortCalibration()
    expect(posts).toEqual([])
    expect(r.stopped).toEqual([])
  })

  it('stops the floor sweep, the D435 capture and the CSI capture of its camera', async () => {
    const posts = fakeNode({ sweep: true, d435: true, csi: 'back' })
    const r = await abortCalibration()
    expect(posts.sort()).toEqual([
      '/api/calibration/rgbd/capture/stop',
      '/api/calibration/rgbd/csi/back/stop',
      '/api/calibration/rgbd/floor/sweep/stop',
    ])
    expect(r.stopped.sort()).toEqual(['D435 capture', 'back camera capture', 'floor sweep'])
  })

  it('a state that cannot be read is reported and the others are still stopped', async () => {
    const posts = fakeNode({ sweep: true }, ['/head-arm/'])
    const r = await abortCalibration()
    expect(posts).toEqual(['/api/calibration/rgbd/floor/sweep/stop'])
    expect(r.errors).toEqual([expect.stringContaining('arm-camera capture')])
  })
})
