// @vitest-environment jsdom
import { afterEach, describe, expect, it, vi } from 'vitest'
import { act, cleanup, fireEvent, render, screen, waitFor, within } from '@testing-library/react'
import { CameraGroundFlow, GROUND_POLL_MS } from './CameraGroundFlow'
import type { GroundResult, GroundState, GroundSweep, GroundVerifyState } from '../groundBoardApi'
import {
  doneResult,
  doneSweep,
  failedVerify,
  idleState,
  nodeAccept,
  nodeRunning,
  nodeState,
  nodeSweep,
  nodeVerify,
  refusedResult,
  sweepingSweep,
} from '../groundBoardFixtures'

vi.mock('../RgbdCalibration', () => ({ RgbdCalibration: () => <div>d435-check</div> }))
vi.mock('../CsiIntrinsics', () => ({ CsiIntrinsics: () => <div>csi-cameras</div> }))
vi.mock('../BoardNeeded', () => ({ BoardNeeded: ({ role }: { role: string }) => <div>{`board-${role}`}</div> }))
vi.mock('../FloorFitMore', () => ({
  FloorFitMore: ({ disabled }: { disabled?: boolean }) => <div>{`more-fits${disabled ? ' disabled' : ''}`}</div>,
}))

const B = '/api/calibration/rgbd/ground-board'
const SID = nodeSweep.session_id as string
const RID = 'r-20261109-040000'
const PREVIEW = { topic: '/head/camera/image', data: { jpeg_b64: 'AAAA' } }

interface World {
  state: GroundState
  /** Result bodies served in order (the last one repeats). */
  results: GroundResult[]
  calls: string[]
  /** Path suffix answered 409 with `refusal`. */
  refuse?: string
  refusal?: string
  home: { ok: boolean; message: string }
}

function install(w: World): void {
  vi.stubGlobal(
    'fetch',
    vi.fn(async (url: string, init?: RequestInit) => {
      const method = init?.method ?? 'GET'
      w.calls.push(`${method} ${url}`)
      const json = (b: unknown, status = 200) => new Response(JSON.stringify(b), { status })
      if (w.refuse && url.endsWith(w.refuse)) return json({ ok: false, message: w.refusal ?? 'refused' }, 409)
      if (url === '/api/calibration/general/home_head') return json(w.home)
      if (url === `${B}/state`) return json(w.state)
      const sweep = (s: GroundSweep) => {
        w.state = { ...w.state, sweep: s }
        return json(s)
      }
      const verify = (v: GroundVerifyState) => {
        w.state = { ...w.state, verify: v }
        return json(v)
      }
      if (url === `${B}/sweep/start`) return sweep(sweepingSweep)
      if (url === `${B}/sweep/next-placement`) return sweep({ ...sweepingSweep, placement: 1, placements_done: 1, pose: 1 })
      if (url === `${B}/sweep/stop`) return sweep(doneSweep)
      if (url === `${B}/sessions/${SID}/compute`) return json({ result_id: RID }, 202)
      if (url === `${B}/sessions/${SID}/results/${RID}`) return json(w.results.length > 1 ? w.results.shift() : w.results[0])
      if (url.endsWith('/accept')) return json(nodeAccept)
      if (url.endsWith('/decline')) return json({ ...w.results[0], declined: true, declined_at: '2026-11-09T05:00:00+00:00' })
      if (url === `${B}/verify/start`) return verify({ ...nodeVerify, running: true, status: 'measuring', passed: null, poses: [] })
      if (url === `${B}/verify/stop`) return verify({ ...nodeVerify, running: false, status: 'stopped', passed: null })
      return json({ ok: false, message: 'nope' }, 404)
    }),
  )
}

const world = (over: Partial<World> = {}): World => ({
  state: idleState,
  results: [{ ...doneResult, id: RID, session: SID }],
  calls: [],
  home: { ok: true, message: 'head homed' },
  ...over,
})

/** The state of a finished session of SID. */
const sessionDone = (over: Partial<GroundState> = {}): GroundState => ({ ...idleState, sweep: doneSweep, ...over })

const steps = () => within(screen.getByRole('list', { name: 'Camera to ground calibration steps' })).getAllByRole('listitem')
const step = (n: number) => within(steps()[n - 1])

function renderFlow(props: Partial<Parameters<typeof CameraGroundFlow>[0]> = {}) {
  return render(<CameraGroundFlow canHomeHead headCamera={null} {...props} />)
}

async function confirm(label: string): Promise<void> {
  await act(async () => {
    fireEvent.click(within(screen.getByRole('dialog')).getByRole('button', { name: label }))
  })
  await waitFor(() => expect(screen.queryByRole('dialog')).toBeNull())
}

/** Compute on a finished session; resolves once the gate shows. */
async function computeDone(): Promise<void> {
  const btn = await screen.findByRole('button', { name: 'Compute' })
  await waitFor(() => expect(btn.hasAttribute('disabled')).toBe(false))
  await act(async () => {
    fireEvent.click(btn)
  })
  await screen.findByText(/^(Plausible|Not plausible)$/, undefined, { timeout: 4000 })
}

afterEach(() => {
  cleanup()
  vi.unstubAllGlobals()
  vi.useRealTimers()
})

describe('CameraGroundFlow', () => {
  it('orders the steps: D435 check, front/back cameras, home, board, sweep, compute, gate, verification', async () => {
    install(world())
    renderFlow()
    await screen.findByText('d435-check')
    expect(steps().map((li) => li.querySelector('h3')?.textContent)).toEqual([
      '1. D435 intrinsics check',
      '2. Front/back cameras',
      '3. Home camera',
      '4. Lay the board',
      '5. Board sweep',
      '6. Compute',
      '7. Check and accept',
      '8. Verify on the ground',
    ])
    expect(step(2).getByText('csi-cameras')).toBeTruthy()
    expect(step(4).getByText('Finish step 3 first.')).toBeTruthy()
    expect(screen.getByText('more-fits')).toBeTruthy()
  })

  it('Home camera confirms, runs the General home_head action and unlocks the board step', async () => {
    const w = world()
    install(w)
    renderFlow()
    fireEvent.click(await screen.findByRole('button', { name: 'Home camera' }))
    await confirm('Home camera')
    expect(w.calls).toContain('POST /api/calibration/general/home_head')
    await waitFor(() => expect(step(3).getByText('Done')).toBeTruthy())
    expect(step(4).getByText('board-full')).toBeTruthy()
  })

  it('a refused Home camera shows the message and keeps the step open', async () => {
    install(world({ home: { ok: false, message: 'head torque is off' } }))
    renderFlow()
    fireEvent.click(await screen.findByRole('button', { name: 'Home camera' }))
    await confirm('Home camera')
    await screen.findByText('head torque is off')
    expect(step(4).getByText('Finish step 3 first.')).toBeTruthy()
  })

  it('without the General Home camera the step is optional and points to the General section', async () => {
    install(world())
    renderFlow({ canHomeHead: false })
    await screen.findByText(/Home the camera from the Head tab/)
    expect(screen.queryByRole('button', { name: 'Home camera' })).toBeNull()
    expect(step(3).getByText('Optional')).toBeTruthy()
  })

  it('the board step shows the head camera preview and Board is down unlocks the sweep', async () => {
    install(world())
    renderFlow({ canHomeHead: false, headCamera: PREVIEW })
    const img = await screen.findByRole('img', { name: 'head camera' })
    expect(img.getAttribute('src')).toBe('data:image/jpeg;base64,AAAA')
    expect(step(4).getByText(/flat on the floor, close to the robot, fully visible/)).toBeTruthy()
    fireEvent.click(step(4).getByRole('button', { name: 'Board is down' }))
    expect(step(5).getByRole('button', { name: 'Start sweep' })).toBeTruthy()
  })

  it('without a preview the board step says to check the Head tab', async () => {
    install(world())
    renderFlow({ canHomeHead: false })
    await screen.findByText(/No head camera preview here: check the board in the Head tab/)
  })

  it('Start sweep posts and polls the session while it runs; Stop ends it', async () => {
    vi.useFakeTimers()
    const w = world()
    install(w)
    renderFlow({ canHomeHead: false })
    await act(async () => {})
    fireEvent.click(screen.getByRole('button', { name: 'Board is down' }))
    await act(async () => {
      fireEvent.click(screen.getByRole('button', { name: 'Start sweep' }))
    })
    expect(w.calls).toContain(`POST ${B}/sweep/start`)
    expect(screen.getByText(/Placement 1 of 3: pose 4 of 10, 11 views, 8 depth planes, 0 skipped - sweeping/)).toBeTruthy()
    expect(screen.getByText(/Board located at x 0.551 m, y 0.049 m, yaw 17.2 deg/)).toBeTruthy()
    const before = w.calls.filter((c) => c === `GET ${B}/state`).length
    await act(async () => {
      await vi.advanceTimersByTimeAsync(GROUND_POLL_MS * 2)
    })
    expect(w.calls.filter((c) => c === `GET ${B}/state`).length).toBeGreaterThan(before)
    expect(screen.getByText('more-fits disabled')).toBeTruthy()
    await act(async () => {
      fireEvent.click(screen.getByRole('button', { name: 'Stop' }))
    })
    expect(w.calls).toContain(`POST ${B}/sweep/stop`)
    expect(step(5).getByText('Done')).toBeTruthy()
  })

  it('awaiting a placement shows the hint and Next placement posts', async () => {
    const w = world({ state: { ...idleState, sweep: nodeSweep } })
    install(w)
    renderFlow({ canHomeHead: false, headCamera: PREVIEW })
    await screen.findByText(/move the board to a new place on the floor/)
    expect(screen.getByText(/1 of 3 placements done/)).toBeTruthy()
    expect(step(5).getByRole('img', { name: 'head camera' })).toBeTruthy()
    await act(async () => {
      fireEvent.click(screen.getByRole('button', { name: 'Next placement' }))
    })
    expect(w.calls).toContain(`POST ${B}/sweep/next-placement`)
    expect(screen.getByText(/Placement 2 of 3: pose 1 of 10/)).toBeTruthy()
  })

  it('Next placement is off once every placement is done and while a placement runs', async () => {
    install(world({ state: { ...idleState, sweep: { ...nodeSweep, placements_done: 3 } } }))
    renderFlow({ canHomeHead: false })
    expect((await screen.findByRole('button', { name: 'Next placement' })).hasAttribute('disabled')).toBe(true)
    cleanup()
    install(world({ state: { ...idleState, sweep: sweepingSweep } }))
    renderFlow({ canHomeHead: false })
    expect((await screen.findByRole('button', { name: 'Next placement' })).hasAttribute('disabled')).toBe(true)
  })

  it('a refused start shows the node message on the sweep step', async () => {
    install(world({ refuse: '/sweep/start', refusal: 'a floor sweep is running; stop it first' }))
    renderFlow({ canHomeHead: false })
    fireEvent.click(await screen.findByRole('button', { name: 'Board is down' }))
    await act(async () => {
      fireEvent.click(screen.getByRole('button', { name: 'Start sweep' }))
    })
    expect(step(5).getByText('a floor sweep is running; stop it first')).toBeTruthy()
  })

  it('a finished session from the node opens Compute; the result is polled until done and gated', async () => {
    const w = world({ state: sessionDone(), results: [{ ...nodeRunning, id: RID, session: SID }, { ...doneResult, id: RID, session: SID }] })
    install(w)
    renderFlow()
    await computeDone()
    expect(w.calls).toContain(`POST ${B}/sessions/${SID}/compute`)
    expect(w.calls.filter((c) => c === `GET ${B}/sessions/${SID}/results/${RID}`).length).toBeGreaterThan(1)
    const gate = step(7)
    expect(gate.getByText('Plausible')).toBeTruthy()
    expect(gate.getByText(/board placements 1 are fewer than the advisory 2/)).toBeTruthy()
    expect(within(gate.getByRole('row', { name: /^mount_yaw_deg/ })).getByText('held')).toBeTruthy()
    expect(within(gate.getByRole('row', { name: /^floor height 1 m ahead of the head/ })).getByText('+-0.3 mm')).toBeTruthy()
    expect(gate.getByRole('table', { name: 'Board placements' })).toBeTruthy()
  })

  it('a compute that failed shows its error on the compute step', async () => {
    install(world({ state: sessionDone(), results: [{ ...nodeRunning, id: RID, session: SID, state: 'failed', error: 'no views' }] }))
    renderFlow()
    const btn = await screen.findByRole('button', { name: 'Compute' })
    await waitFor(() => expect(btn.hasAttribute('disabled')).toBe(false))
    await act(async () => {
      fireEvent.click(btn)
    })
    await waitFor(() => expect(step(6).getByText('no views')).toBeTruthy())
  })

  it('a refused result cannot be accepted and names the refusal with value against limit', async () => {
    install(world({ state: sessionDone(), results: [{ ...refusedResult, id: RID, session: SID }] }))
    renderFlow()
    await computeDone()
    expect(screen.getByText('Not plausible')).toBeTruthy()
    expect(screen.getByText(/head mount height change 120 mm .*value 120 mm, limit 100 mm/)).toBeTruthy()
    expect(screen.getByRole('button', { name: 'Accept' }).hasAttribute('disabled')).toBe(true)
  })

  it('Accept confirms, applies the head chain and shows the unit restart state', async () => {
    const w = world({ state: sessionDone() })
    install(w)
    renderFlow()
    await computeDone()
    fireEvent.click(screen.getByRole('button', { name: 'Accept' }))
    await confirm('Accept')
    expect(w.calls).toContain(`POST ${B}/sessions/${SID}/results/${RID}/accept`)
    await screen.findByText(/Applied result r-20261109-040000 \(ground board\)\. Units: unit-a.service restarted \(active\)\./)
    expect(screen.getByText('Accepted')).toBeTruthy()
  })

  it('a refused accept shows the node message on the gate', async () => {
    install(world({ state: sessionDone(), refuse: '/accept', refusal: 'no accepted D435 intrinsics check' }))
    renderFlow()
    await computeDone()
    fireEvent.click(screen.getByRole('button', { name: 'Accept' }))
    await confirm('Accept')
    await screen.findByText('no accepted D435 intrinsics check')
    expect(screen.queryByText('Accepted')).toBeNull()
  })

  it('Decline posts the decline and applies nothing', async () => {
    const w = world({ state: sessionDone() })
    install(w)
    renderFlow()
    await computeDone()
    await act(async () => {
      fireEvent.click(screen.getByRole('button', { name: 'Decline' }))
    })
    expect(w.calls).toContain(`POST ${B}/sessions/${SID}/results/${RID}/decline`)
    expect(screen.getByText('Declined')).toBeTruthy()
    expect(w.calls.some((c) => c.endsWith('/accept'))).toBe(false)
  })

  it('shows the head chain in effect and the accepted ground board result with its verification', async () => {
    install(world({ state: { ...nodeState, sweep: doneSweep } }))
    renderFlow()
    await screen.findByText(/Head chain in effect: ground board, session ground-20261109-033000, result r-20261109-040000/)
    expect(screen.getByText(/Accepted ground board result r-20261109-040000 .*RMS 0.42 px.*not verified yet/)).toBeTruthy()
  })

  it('verification is open once a head chain is applied: start posts and polls; pass and fail per pose with values vs limits', async () => {
    const w = world({ state: { ...nodeState, sweep: doneSweep, verify: failedVerify } })
    install(w)
    renderFlow()
    const verify = await screen.findByRole('table', { name: 'Ground verification poses' })
    const rows = within(verify).getAllByRole('row')
    expect(within(rows[1]).getByText('Passed')).toBeTruthy()
    expect(within(rows[1]).getByText(/height 3.0 mm \(limit 10.0 mm\) ok/)).toBeTruthy()
    expect(within(rows[2]).getByText('Failed')).toBeTruthy()
    expect(within(rows[2]).getByText(/height 15.0 mm \(limit 10.0 mm\) failed/)).toBeTruthy()
    expect(step(8).getByText('Verification failed')).toBeTruthy()
    await act(async () => {
      fireEvent.click(step(8).getByRole('button', { name: 'Start verification' }))
    })
    expect(w.calls).toContain(`POST ${B}/verify/start`)
    expect(step(8).getByText(/measuring/)).toBeTruthy()
    await act(async () => {
      fireEvent.click(step(8).getByRole('button', { name: 'Stop verification' }))
    })
    expect(w.calls).toContain(`POST ${B}/verify/stop`)
  })

  it('a passed verification marks the last step done', async () => {
    install(world({ state: { ...nodeState, sweep: doneSweep } }))
    renderFlow()
    await waitFor(() => expect(step(8).getByText('Verification passed')).toBeTruthy())
    expect(step(8).getByText('Done')).toBeTruthy()
  })

  it('verification is locked without an applied head chain', async () => {
    install(world())
    renderFlow()
    await screen.findByText('d435-check')
    expect(step(8).queryByRole('button', { name: 'Start verification' })).toBeNull()
  })

  it('a compute running on the node after a page reload is polled to its result', async () => {
    const compute = { running: true, session_id: SID, result_id: RID, started_at: '2026-11-09T04:00:01+00:00', error: null }
    const w = world({ state: sessionDone({ compute }) })
    install(w)
    renderFlow()
    await screen.findByText('Plausible')
    expect(w.calls).toContain(`GET ${B}/sessions/${SID}/results/${RID}`)
    expect(w.calls).not.toContain(`POST ${B}/sessions/${SID}/compute`)
  })

  it('shows a state the node cannot give as an error', async () => {
    vi.stubGlobal('fetch', vi.fn(async () => new Response(JSON.stringify({ ok: false, message: 'calibration node unreachable' }), { status: 503 })))
    renderFlow()
    await screen.findByText(/calibration node unreachable/)
  })
})
