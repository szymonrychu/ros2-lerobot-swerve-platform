// @vitest-environment jsdom
import { afterEach, describe, expect, it, vi } from 'vitest'
import { act, cleanup, fireEvent, render, screen, waitFor, within } from '@testing-library/react'
import { CameraGroundFlow } from './CameraGroundFlow'
import { FLOOR_POLL_MS } from '../HeadFloorFit'
import stateFx from '../../../../../head_rgbd_calibration/tests/fixtures/api/floor_state.json'
import resultFx from '../../../../../head_rgbd_calibration/tests/fixtures/api/floor_result.json'
import applyFx from '../../../../../head_rgbd_calibration/tests/fixtures/api/floor_apply.json'

import type { SweepState } from '../headFloorApi'

vi.mock('../RgbdCalibration', () => ({ RgbdCalibration: () => <div>d435-check</div> }))
vi.mock('../CsiIntrinsics', () => ({ CsiIntrinsics: () => <div>csi-cameras</div> }))

const IDLE: SweepState = { ...stateFx.sweep, running: false, status: 'done' }
const RUNNING: SweepState = stateFx.sweep
const FAILED = { ...resultFx, passed: false, failed_limits: ['floor.fit.max_height_rms_mm: height RMS 9 mm exceeds 5 mm'] }

interface World {
  sweep: SweepState
  applied: typeof stateFx.applied | null
  result: unknown
  calls: string[]
  failNext?: string
}

function install(w: World): void {
  vi.stubGlobal(
    'fetch',
    vi.fn(async (url: string, init?: RequestInit) => {
      w.calls.push(`${init?.method ?? 'GET'} ${url}`)
      const json = (b: unknown, status = 200) => new Response(JSON.stringify(b), { status })
      if (w.failNext && url.includes(w.failNext)) return json({ ok: false, message: 'compute exploded' }, 500)
      if (url.endsWith('/state')) return json({ sweep: w.sweep, applied: w.applied })
      if (url.endsWith('/sweep/start')) {
        w.sweep = RUNNING
        return json(w.sweep)
      }
      if (url.endsWith('/sweep/stop')) {
        w.sweep = IDLE
        return json(w.sweep)
      }
      if (url.endsWith('/compute')) return json(w.result)
      if (url.endsWith('/apply')) return json(applyFx)
      return json({ message: 'nope' }, 404)
    }),
  )
}

const world = (over: Partial<World> = {}): World => ({ sweep: IDLE, applied: null, result: resultFx, calls: [], ...over })

async function ready(): Promise<HTMLElement> {
  await screen.findByText(/Pose 3 of 15/)
  return screen.getByRole('button', { name: 'Compute' })
}

afterEach(() => {
  cleanup()
  vi.unstubAllGlobals()
  vi.useRealTimers()
})

const steps = () => within(screen.getByRole('list', { name: 'Camera to ground calibration steps' })).getAllByRole('listitem')

/** Accepts the shown floor result through the gate confirm. */
async function accept(): Promise<void> {
  fireEvent.click(screen.getByRole('button', { name: 'Accept' }))
  await act(async () => {
    fireEvent.click(within(screen.getByRole('dialog')).getByRole('button', { name: 'Accept' }))
  })
  await waitFor(() => expect(screen.queryByRole('dialog')).toBeNull())
}

describe('CameraGroundFlow', () => {
  it('orders the steps: D435 check, front/back cameras, sweep, compute, gate, verification', async () => {
    install(world({ sweep: { ...IDLE, session_id: null } }))
    render(<CameraGroundFlow />)
    await screen.findByText('d435-check')
    expect(steps().map((li) => li.querySelector('h3')?.textContent)).toEqual([
      '1. D435 intrinsics check',
      '2. Front/back cameras',
      '3. Head floor sweep',
      '4. Compute',
      '5. Check and apply',
      '6. Verify on the ground (coming)',
    ])
    expect(within(steps()[1]).getByText('csi-cameras')).toBeTruthy()
  })

  it('shows the precondition text and Start sweep', async () => {
    install(world({ sweep: { ...IDLE, session_id: null } }))
    render(<CameraGroundFlow />)
    await screen.findByText('3. Head floor sweep')
    expect(screen.getByText(/flat floor, not on blocks/)).toBeTruthy()
    expect(screen.getByText(/2 m in front and 1.5 m to each side/)).toBeTruthy()
    expect(screen.getByText(/only the head moves/i)).toBeTruthy()
    expect(within(steps()[3]).getByText('Finish step 3 first.')).toBeTruthy()
  })

  it('says the floor plane comes from the D435 depth (D435 wording)', async () => {
    install(world({ sweep: { ...IDLE, session_id: null } }))
    const { container } = render(<CameraGroundFlow />)
    await screen.findByText('3. Head floor sweep')
    expect(screen.getByText(/floor plane from the D435 depth/)).toBeTruthy()
    expect(container.textContent).not.toMatch(/ToF/)
  })

  it('start posts, then progress polls while running and Stop ends it', async () => {
    vi.useFakeTimers()
    const w = world({ sweep: { ...IDLE, session_id: null } })
    install(w)
    render(<CameraGroundFlow />)
    await act(async () => {})
    await act(async () => {
      fireEvent.click(screen.getByRole('button', { name: 'Start sweep' }))
    })
    expect(w.calls).toContain('POST /api/calibration/rgbd/floor/sweep/start')
    expect(screen.getByText(/Pose 3 of 15/)).toBeTruthy()
    expect(screen.getByText(/12 observations/)).toBeTruthy()
    expect(screen.getByText(/1 skipped/)).toBeTruthy()
    const before = w.calls.filter((c) => c.endsWith('/state')).length
    await act(async () => {
      await vi.advanceTimersByTimeAsync(FLOOR_POLL_MS * 2)
    })
    expect(w.calls.filter((c) => c.endsWith('/state')).length).toBeGreaterThan(before)
    await act(async () => {
      fireEvent.click(screen.getByRole('button', { name: 'Stop' }))
    })
    expect(w.calls).toContain('POST /api/calibration/rgbd/floor/sweep/stop')
    expect(screen.getByRole('button', { name: 'Compute' }).hasAttribute('disabled')).toBe(false)
  })

  it('compute shows the table with current, fitted, delta, sigma and the residuals', async () => {
    const w = world()
    install(w)
    render(<CameraGroundFlow />)
    const btn = await ready()
    await act(async () => {
      fireEvent.click(btn)
    })
    expect(w.calls).toContain('POST /api/calibration/rgbd/floor/sessions/floor-20261021-120000/compute')
    const row = (name: string) => within(screen.getByRole('row', { name: new RegExp(name) }))
    // mount z: 350 -> 380 mm, change +30, sigma 0.4 mm
    expect(row('Mount z').getByText('350.0')).toBeTruthy()
    expect(row('Mount z').getByText('380.0')).toBeTruthy()
    expect(row('Mount z').getByText('+30.0')).toBeTruthy()
    expect(row('Mount z').getByText('0.4')).toBeTruthy()
    expect(row('Mount roll').getByText('1.500')).toBeTruthy()
    expect(row('Mount pitch').getAllByText('-3.000')).toHaveLength(2)
    expect(row('Tilt zero').getByText('+2.000')).toBeTruthy()
    expect(row('Tilt roll').getByText('+1.200')).toBeTruthy()
    expect(row('Tilt roll').getByText('0.030')).toBeTruthy()
    expect(screen.getByText(/Checked against the limits \(pose mean\): Height RMS 2.1 mm, Normal RMS 0.12 deg/)).toBeTruthy()
    expect(screen.getByText(/Per observation \(not limited\): Height RMS 4.3 mm, Normal RMS 0.9 deg/)).toBeTruthy()
    expect(screen.getByText('Plausible')).toBeTruthy()
    expect(screen.getByText(/camera height: \+-0.4 mm/)).toBeTruthy()
    // per-pose residual and skipped reason
    expect(screen.getByRole('row', { name: /-60.0.*12.0.*3.*1.9.*0.10/ })).toBeTruthy()
    expect(screen.getByText(/no valid plane/)).toBeTruthy()
  })

  it('shows a partly blocked skipped pose with its reason', async () => {
    const reason = 'partly blocked: 2 of 5 clouds'
    install(world({ result: { ...resultFx, skipped: [{ pose: 4, pan_deg: 30, tilt_deg: 10, reason }] } }))
    render(<CameraGroundFlow />)
    const compute = await ready()
    await act(async () => {
      fireEvent.click(compute)
    })
    expect(screen.getByText(new RegExp(`Skipped pose 4.*${reason}`))).toBeTruthy()
  })

  it('renders an older result without tilt roll and observation RMS', async () => {
    const { tilt_roll_deg: _t, ...params } = resultFx.params
    const { observation_height_rms_mm: _h, observation_normal_rms_deg: _n, ...old } = resultFx
    install(world({ result: { ...old, params } }))
    render(<CameraGroundFlow />)
    const compute = await ready()
    await act(async () => {
      fireEvent.click(compute)
    })
    expect(screen.getByText('Plausible')).toBeTruthy()
    expect(screen.queryByRole('row', { name: /Tilt roll/ })).toBeNull()
    expect(screen.queryByText(/Per observation/)).toBeNull()
    expect(screen.getByText(/pose mean\): Height RMS 2.1 mm/)).toBeTruthy()
  })

  it('Accept is disabled for a failed result and shows the failed limits', async () => {
    install(world({ result: FAILED }))
    render(<CameraGroundFlow />)
    const compute = await ready()
    await act(async () => {
      fireEvent.click(compute)
    })
    expect(screen.getByText('Not plausible')).toBeTruthy()
    expect(screen.getByText(/height RMS 9 mm exceeds 5 mm/)).toBeTruthy()
    expect(screen.getByRole('button', { name: 'Accept' }).hasAttribute('disabled')).toBe(true)
  })

  it('Accept for a passed result confirms, applies and shows the unit restart state', async () => {
    const w = world()
    install(w)
    render(<CameraGroundFlow />)
    const compute = await ready()
    await act(async () => {
      fireEvent.click(compute)
    })
    await accept()
    expect(w.calls).toContain('POST /api/calibration/rgbd/floor/sessions/floor-20261021-120000/results/r-20261021-123000/apply')
    expect(screen.getByText(/ros2-head_camera_cpp.service/)).toBeTruthy()
    expect(screen.getByText(/restarted/)).toBeTruthy()
    expect(screen.getByText('Accepted')).toBeTruthy()
  })

  it('Decline applies nothing', async () => {
    const w = world()
    install(w)
    render(<CameraGroundFlow />)
    const compute = await ready()
    await act(async () => {
      fireEvent.click(compute)
    })
    fireEvent.click(screen.getByRole('button', { name: 'Decline' }))
    expect(screen.getByText('Declined')).toBeTruthy()
    expect(w.calls.some((c) => c.endsWith('/apply'))).toBe(false)
  })

  it('shows the currently applied head chain', async () => {
    install(world({ applied: stateFx.applied }))
    render(<CameraGroundFlow />)
    await screen.findByText(/Applied head chain/)
    expect(screen.getByText(/floor-20261021-120000/)).toBeTruthy()
    expect(screen.getByText(/r-20261021-123000/)).toBeTruthy()
    expect(screen.getByText(/height RMS 2.1 mm, normal RMS 0.12 deg/)).toBeTruthy()
  })

  it('shows errors from the node and from a malformed body', async () => {
    const w = world({ failNext: '/compute' })
    install(w)
    render(<CameraGroundFlow />)
    const compute = await ready()
    await act(async () => {
      fireEvent.click(compute)
    })
    expect(screen.getByText('compute exploded')).toBeTruthy()
    cleanup()
    install(world({ result: { id: 'x' } }))
    render(<CameraGroundFlow />)
    const compute2 = await ready()
    await act(async () => {
      fireEvent.click(compute2)
    })
    expect(screen.getByText('Unexpected floor fit response from the robot')).toBeTruthy()
    expect(screen.queryByText('Plausible')).toBeNull()
  })

  it('shows a sweep error from the node state', async () => {
    install(world({ sweep: { ...IDLE, status: 'error', error: 'head did not reach the pose' } }))
    render(<CameraGroundFlow />)
    await screen.findByText(/head did not reach the pose/)
  })
})
