// @vitest-environment jsdom
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { act, cleanup, fireEvent, render, screen, waitFor, within } from '@testing-library/react'
import { ArmCameraFlow, RESULT_POLL_MS } from './ArmCameraFlow'
import { ARM_POLL_MS } from '../SequenceActivity'
import stateFx from '../../../../../head_rgbd_calibration/tests/fixtures/api/head_arm_state.json'
import rgbdStateFx from '../../../../../head_rgbd_calibration/tests/fixtures/api/state.json'
import armStateFx from '../fixtures/arm_camera_state.json'
import doneFx from '../fixtures/arm_camera_result_done.json'
import refusedFx from '../fixtures/arm_camera_result_refused.json'
import runningFx from '../fixtures/arm_camera_result_running.json'
import * as api from '../headArmApi'
import * as armApi from '../armCameraApi'
import * as generalApi from '../generalApi'
import type { HeadArmSequence } from '../headArmApi'
import type { ArmCameraResult, ArmCameraState } from '../armCameraApi'
import type { ServoGroup } from '../servoApi'

vi.mock('../headArmApi')
vi.mock('../armCameraApi')
const m = vi.mocked(api)
const a = vi.mocked(armApi)

const ok = <T,>(value: T) => ({ ok: true as const, value })
const seq = (over: Partial<HeadArmSequence>): HeadArmSequence => ({
  ...stateFx.sequence,
  running: false,
  phase: 'idle',
  status: 'idle',
  session_id: null,
  board_record: null,
  skipped: {},
  events: [],
  ...over,
})
const SID = 'calib-20261109-113000'
const HOLDING = seq({ phase: 'holding', session_id: SID, captured: 40, status: 'holding the board' })
const SESSION = { id: SID, created: 'x', views: 40, arm_poses: 12, results: [] as string[] }
const MODELS = {
  files: [
    { name: 'charuco-arm-6x5-28mm.3mf', size: 1, url: '/models/charuco-arm-6x5-28mm.3mf' },
    { name: 'charuco-arm-6x5-28mm.pdf', size: 1, url: '/models/charuco-arm-6x5-28mm.pdf' },
  ],
}

let servoGroups: ServoGroup[]

function armGroup(stale: string[] | null = null): ServoGroup {
  return {
    id: 'arm',
    label: 'Arm',
    neutral: true,
    state: null,
    age_s: null,
    stale: true,
    calibration_stale: stale,
    zero_restore: true,
    move_to_neutral: true,
  }
}

/** BoardNeeded reads the node state and the model files; the arm zero check reads the servo groups. */
function stubFetch(): void {
  vi.stubGlobal(
    'fetch',
    vi.fn(async (url: string) => {
      if (url === '/api/models') return new Response(JSON.stringify(MODELS))
      if (url === '/api/calibration/rgbd/state') return new Response(JSON.stringify(rgbdStateFx))
      if (url === '/api/calibration/servos') return new Response(JSON.stringify({ groups: servoGroups }))
      return new Response('{}', { status: 404 })
    }),
  )
}

function setState(s: HeadArmSequence, armState: ArmCameraState = armStateFx as ArmCameraState): void {
  m.getHeadArmState.mockResolvedValue(ok({ sequence: s, applied: null }))
  m.listHeadArmSessions.mockResolvedValue(ok([SESSION]))
  a.getArmCameraState.mockResolvedValue(ok(armState))
}

const click = async (name: string | RegExp, root: HTMLElement = document.body) => {
  await act(async () => {
    fireEvent.click(within(root).getByRole('button', { name }))
  })
}
const dialog = () => screen.getByRole('dialog')
const confirmIn = async (name: string | RegExp) => {
  await click(name, dialog())
  await waitFor(() => expect(screen.queryByRole('dialog')).toBeNull())
}
const steps = () => within(screen.getByRole('list', { name: 'Arm to camera calibration steps' })).getAllByRole('listitem')
const step = (n: number) => steps()[n - 1]

async function renderFlow(canHomeArm = true): Promise<void> {
  render(<ArmCameraFlow canHomeArm={canHomeArm} />)
  await act(async () => {})
}

beforeEach(() => {
  vi.resetAllMocks()
  servoGroups = [armGroup()]
  stubFetch()
})
afterEach(() => {
  cleanup()
  vi.useRealTimers()
  vi.unstubAllGlobals()
  vi.restoreAllMocks()
})

describe('ArmCameraFlow', () => {
  it('names the seven steps in order', async () => {
    setState(seq({}))
    await renderFlow()
    expect(steps().map((li) => li.querySelector('h3')?.textContent)).toEqual([
      '1. Home arm',
      '2. Open gripper',
      '3. Grab board',
      '4. Capture',
      '5. Compute',
      '6. Check and accept',
      '7. Park',
    ])
  })

  it('says the D435 watches the ARM board held by its handle and names the board', async () => {
    setState(seq({}))
    const { container } = render(<ArmCameraFlow canHomeArm />)
    expect(await screen.findByText(/ARM board: charuco-arm-6x5-28mm, 6x5 squares of 28 mm/)).toBeTruthy()
    expect(screen.getByText(/the head D435 watches the ARM board held by its handle in the jaws/)).toBeTruthy()
    expect(container.textContent).not.toMatch(/ToF/)
  })

  it('a not-ready state blocks the arm moves with the node reason', async () => {
    setState(seq({}), { applied: null, head_chain_source: null, ready: false, reason: 'apply a camera-to-ground calibration first' })
    await renderFlow(false)
    expect(screen.getByText(/apply a camera-to-ground calibration first/)).toBeTruthy()
    expect((within(step(2)).getByRole('button', { name: 'Open gripper' }) as HTMLButtonElement).disabled).toBe(true)
  })

  it('a stale arm zero blocks the arm moves', async () => {
    servoGroups = [armGroup(['joint_offsets'])]
    setState(seq({}))
    await renderFlow()
    expect(await screen.findByText(/Arm zero is stale/)).toBeTruthy()
    expect((within(step(1)).getByRole('button', { name: 'Home arm' }) as HTMLButtonElement).disabled).toBe(true)
  })

  it('an unreadable fit state is shown but does not block the capture', async () => {
    setState(seq({}))
    a.getArmCameraState.mockResolvedValue({ ok: false, status: 404, message: 'HTTP 404' })
    await renderFlow()
    expect(screen.getByText(/Arm-to-camera fit state unavailable: HTTP 404/)).toBeTruthy()
    expect((within(step(1)).getByRole('button', { name: 'Home arm' }) as HTMLButtonElement).disabled).toBe(false)
  })

  it('Home arm confirms, runs the general home and unlocks step 2', async () => {
    setState(seq({}))
    const home = vi.spyOn(generalApi, 'runGeneralAction').mockResolvedValue(ok({ ok: true, message: 'converged: homed' }))
    await renderFlow()
    expect(within(step(2)).getByText('Finish step 1 first.')).toBeTruthy()
    await click('Home arm', step(1))
    expect(home).not.toHaveBeenCalled()
    await confirmIn('Home arm')
    expect(home).toHaveBeenCalledWith('home_arm')
    expect(within(step(1)).getByText('Done')).toBeTruthy()
    expect(within(step(2)).getByRole('button', { name: 'Open gripper' })).toBeTruthy()
  })

  it('a refused home shows the message on its step', async () => {
    setState(seq({}))
    vi.spyOn(generalApi, 'runGeneralAction').mockResolvedValue({ ok: false, status: 409, message: 'Arm servo torque is off' })
    await renderFlow()
    await click('Home arm', step(1))
    await confirmIn('Home arm')
    expect(within(step(1)).getByText('Arm servo torque is off')).toBeTruthy()
    expect(within(step(2)).getByText('Finish step 1 first.')).toBeTruthy()
  })

  it('without the general home the first step is optional', async () => {
    setState(seq({}))
    await renderFlow(false)
    expect(within(step(1)).getByText(/Home the arm from the Arm tab/)).toBeTruthy()
    expect(within(step(2)).getByRole('button', { name: 'Open gripper' })).toBeTruthy()
  })

  it('Open gripper moves to the base pose only after the confirm naming the joints', async () => {
    setState(seq({}))
    m.moveToBase.mockResolvedValue(ok(seq({ running: true, phase: 'moving_to_base' })))
    await renderFlow(false)
    await click('Open gripper', step(2))
    const body = within(dialog()).getByText(/These joints move/).textContent
    for (const j of ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']) expect(body).toContain(j)
    await click('Cancel', dialog())
    expect(m.moveToBase).not.toHaveBeenCalled()
    await waitFor(() => expect(screen.queryByRole('dialog')).toBeNull())
    await click('Open gripper', step(2))
    await confirmIn('Move')
    expect(m.moveToBase).toHaveBeenCalledWith(true)
  })

  it('a node error of the base move shows on its step', async () => {
    setState(seq({}))
    m.moveToBase.mockResolvedValue({ ok: false, status: 409, message: 'arm busy' })
    await renderFlow(false)
    await click('Open gripper', step(2))
    await confirmIn('Move')
    expect(within(step(2)).getByText('arm busy')).toBeTruthy()
  })

  it('Grab board: Board attached posts once the arm waits for the board', async () => {
    setState(seq({ phase: 'waiting_board', running: true, session_id: SID }))
    m.boardAttached.mockResolvedValue(ok(seq({ running: true, phase: 'capturing', session_id: SID })))
    await renderFlow(false)
    expect(within(step(2)).getByText('Done')).toBeTruthy()
    expect(within(step(3)).getByText(/Seat the arm board handle in the gripper jaws up to the shoulder/)).toBeTruthy()
    await click('Board attached', step(3))
    expect(m.boardAttached).toHaveBeenCalled()
  })

  it('Capture shows the tier progress, the skip reasons and Stop', async () => {
    setState(seq({ running: true, phase: 'capturing', session_id: SID, tier: 1, arm_pose: 7, total_poses: 24, captured: 30, status: 'capturing tier 1', skipped: { 'inside D435 min-Z': 3, 'outside colour FOV': 2 } }))
    m.stopSequence.mockResolvedValue(ok(HOLDING))
    await renderFlow(false)
    expect(within(step(4)).getByText(/Tier 1, pose 8 of 24, 30 captured, 5 skipped - capturing tier 1/)).toBeTruthy()
    expect(within(step(4)).getByText('inside D435 min-Z: 3')).toBeTruthy()
    expect(within(step(4)).getByText('outside colour FOV: 2')).toBeTruthy()
    await click('Stop', step(4))
    expect(m.stopSequence).toHaveBeenCalled()
  })

  it('a sequence with null tier and arm_pose shows no null text', async () => {
    setState(seq({ running: true, phase: 'capturing', session_id: SID, tier: null, arm_pose: null }))
    await renderFlow(false)
    expect(within(step(4)).getByText(/Tier -, pose - of/)).toBeTruthy()
    expect(screen.queryByText(/null/)).toBeNull()
  })

  it('the shoulder and elbow moves need their consent; Skip declines them', async () => {
    setState(seq({ running: true, phase: 'waiting_big_joints', session_id: SID }))
    m.allowBigJoints.mockResolvedValue(ok(seq({ running: true, phase: 'capturing', session_id: SID })))
    await renderFlow(false)
    await click('Allow shoulder and elbow moves', step(4))
    expect(within(dialog()).getByText(/elbow_flex, shoulder_lift, shoulder_pan/)).toBeTruthy()
    expect(m.allowBigJoints).not.toHaveBeenCalled()
    await confirmIn('Allow')
    expect(m.allowBigJoints).toHaveBeenLastCalledWith(true)
    setState(seq({ running: true, phase: 'waiting_big_joints', session_id: SID }))
    cleanup()
    await renderFlow(false)
    await click('Skip them', step(4))
    expect(m.allowBigJoints).toHaveBeenLastCalledWith(false)
  })

  it('shows the sequence error banner and the recent events', async () => {
    const events = [{ time: '2026-11-09T10:02:00+00:00', phase: 'capturing', level: 'error', message: 'sequence error: tool failed' }]
    setState(seq({ error: 'tool failed', status: 'error: tool failed', events }))
    await renderFlow(false)
    expect(screen.getByText('Last head-arm error')).toBeTruthy()
    const log = screen.getByRole('list', { name: 'Recent head-arm events' })
    expect(within(log).getByText(/sequence error: tool failed/)).toBeTruthy()
  })

  it('polls the sequence while it runs', async () => {
    vi.useFakeTimers()
    setState(seq({ running: true, phase: 'capturing', session_id: SID }))
    render(<ArmCameraFlow canHomeArm={false} />)
    await act(async () => {})
    const n = m.getHeadArmState.mock.calls.length
    await act(async () => {
      await vi.advanceTimersByTimeAsync(ARM_POLL_MS * 2)
    })
    expect(m.getHeadArmState.mock.calls.length).toBeGreaterThan(n)
  })

  it('Compute starts the fit, polls the result and opens the gate; Accept confirms and accepts', async () => {
    setState(HOLDING)
    a.computeArmCamera.mockResolvedValue(ok({ result_id: doneFx.id }))
    a.getArmCameraResult.mockResolvedValueOnce(ok(runningFx as ArmCameraResult)).mockResolvedValue(ok(doneFx as ArmCameraResult))
    a.acceptArmCamera.mockResolvedValue(ok({ ok: true, message: 'applied' }))
    vi.useFakeTimers({ shouldAdvanceTime: true })
    await renderFlow(false)
    expect(within(step(4)).getByText('Done')).toBeTruthy()
    await click('Compute', step(5))
    expect(a.computeArmCamera).toHaveBeenCalledWith(SID)
    expect(within(step(5)).getByText(/Computing/)).toBeTruthy()
    await act(async () => {
      await vi.advanceTimersByTimeAsync(RESULT_POLL_MS * 2)
    })
    expect(a.getArmCameraResult).toHaveBeenCalledWith(SID, doneFx.id)
    expect(within(step(6)).getByText('Plausible')).toBeTruthy()
    expect(within(step(6)).getByText('tool point at 30 cm reach: +-1.9 mm')).toBeTruthy()
    await click('Accept', step(6))
    expect(a.acceptArmCamera).not.toHaveBeenCalled()
    await confirmIn('Accept')
    expect(a.acceptArmCamera).toHaveBeenCalledWith(SID, doneFx.id)
    expect(within(step(6)).getByText('Accepted')).toBeTruthy()
    expect(within(step(6)).getByText('applied')).toBeTruthy()
  })

  it('an accept refusal (409) shows the node message and keeps the gate open', async () => {
    setState(HOLDING)
    a.computeArmCamera.mockResolvedValue(ok({ result_id: doneFx.id }))
    a.getArmCameraResult.mockResolvedValue(ok(doneFx as ArmCameraResult))
    a.acceptArmCamera.mockResolvedValue({ ok: false, status: 409, message: 'result is not plausible' })
    await renderFlow(false)
    await click('Compute', step(5))
    await waitFor(() => expect(within(step(6)).getByText('Plausible')).toBeTruthy())
    await click('Accept', step(6))
    await confirmIn('Accept')
    expect(within(step(6)).getByText('result is not plausible')).toBeTruthy()
    expect((within(step(6)).getByRole('button', { name: 'Decline' }) as HTMLButtonElement).disabled).toBe(false)
  })

  it('a refused report disables Accept; Decline declines and unlocks Park', async () => {
    setState(HOLDING)
    a.computeArmCamera.mockResolvedValue(ok({ result_id: refusedFx.id }))
    a.getArmCameraResult.mockResolvedValue(ok(refusedFx as ArmCameraResult))
    a.declineArmCamera.mockResolvedValue(ok({}))
    m.park.mockResolvedValue(ok(seq({ running: true, phase: 'parking' })))
    await renderFlow(false)
    await click('Compute', step(5))
    await waitFor(() => expect(within(step(6)).getByText('Not plausible')).toBeTruthy())
    expect((within(step(6)).getByRole('button', { name: 'Accept' }) as HTMLButtonElement).disabled).toBe(true)
    expect(within(step(6)).getByText('held-out tool error 9.4 mm above the 5 mm limit')).toBeTruthy()
    await click('Decline', step(6))
    expect(a.declineArmCamera).toHaveBeenCalledWith(SID, refusedFx.id)
    expect(within(step(6)).getByText('Declined')).toBeTruthy()
    await click('Park arm', step(7))
    expect(m.park).not.toHaveBeenCalled()
    await confirmIn('Park')
    expect(m.park).toHaveBeenCalledWith(true)
  })

  it('a failed fit shows its error on the compute step', async () => {
    setState(HOLDING)
    a.computeArmCamera.mockResolvedValue(ok({ result_id: 'r1' }))
    a.getArmCameraResult.mockResolvedValue(ok({ ...(runningFx as ArmCameraResult), state: 'failed', error: 'too few views' }))
    await renderFlow(false)
    await click('Compute', step(5))
    await waitFor(() => expect(within(step(5)).getByText('too few views')).toBeTruthy())
    expect(within(step(6)).getByText('Finish step 5 first.')).toBeTruthy()
  })

  it('a refused compute start shows the node message', async () => {
    setState(HOLDING)
    a.computeArmCamera.mockResolvedValue({ ok: false, status: 409, message: 'a compute runs already' })
    await renderFlow(false)
    await click('Compute', step(5))
    expect(within(step(5)).getByText('a compute runs already')).toBeTruthy()
  })

  it('the head-from-arm and arm offsets fits stay reachable below the steps', async () => {
    setState(HOLDING)
    await renderFlow(false)
    await click('More fits on this session')
    expect(screen.getByRole('button', { name: 'Compute head from arm' })).toBeTruthy()
    expect(screen.getByRole('button', { name: 'Compute arm offsets' })).toBeTruthy()
  })

  it('Park with the board (lost sequence) states the board is on and shows the remembered record', async () => {
    setState(seq({ board_record: { session_id: 'calib-20261023-074418', clipped_at: '2026-10-23T07:50:00+00:00' } }))
    m.park.mockResolvedValue(ok(seq({ running: true, phase: 'parking' })))
    await renderFlow(false)
    await click('Park (board is on the gripper)', step(1))
    expect(within(dialog()).getByText(/attached in session calib-20261023-074418/)).toBeTruthy()
    expect(m.park).not.toHaveBeenCalled()
    await confirmIn('Park with the board')
    expect(m.park).toHaveBeenCalledWith(true, true)
  })

  it('Home arm is off once the sequence holds or moves the arm', async () => {
    for (const s of [seq({ running: true, phase: 'capturing', session_id: SID }), seq({ phase: 'holding', session_id: SID, captured: 0 })]) {
      setState(s)
      await renderFlow()
      expect((within(step(1)).getByRole('button', { name: 'Home arm' }) as HTMLButtonElement).disabled).toBe(true)
      cleanup()
    }
  })

  it('Park is reachable whenever the arm holds the board, also with nothing captured', async () => {
    setState(seq({ phase: 'holding', session_id: SID, captured: 0, error: 'the gripper closed past the handle contact' }))
    m.park.mockResolvedValue(ok(seq({ running: true, phase: 'parking', session_id: SID })))
    await renderFlow(false)
    await click('Park arm', step(7))
    await confirmIn('Park')
    expect(m.park).toHaveBeenCalledWith(true)
  })

  it('a park that fails after it started is not done', async () => {
    setState(seq({ phase: 'holding', session_id: SID, captured: 0 }))
    m.park.mockResolvedValue(ok(seq({ running: true, phase: 'parking', session_id: SID })))
    await renderFlow(false)
    await click('Park arm', step(7))
    await confirmIn('Park')
    setState(seq({ phase: 'holding', session_id: SID, captured: 0, error: 'park refused: board below the floor margin' }))
    m.park.mockResolvedValue(ok(seq({ phase: 'holding', session_id: SID, error: 'park refused: board below the floor margin' })))
    vi.useFakeTimers({ shouldAdvanceTime: true })
    await act(async () => {
      await vi.advanceTimersByTimeAsync(ARM_POLL_MS * 2)
    })
    expect(within(step(7)).queryByText('Done')).toBeNull()
    expect(within(step(7)).queryByRole('button', { name: 'Start a new run' })).toBeNull()
  })

  it('a finished park is done and a new run starts clean', async () => {
    setState(seq({ phase: 'holding', session_id: SID, captured: 40 }))
    m.park.mockResolvedValue(ok(seq({ phase: 'idle', session_id: SID, captured: 40 })))
    await renderFlow(false)
    await click('Park arm', step(7))
    await confirmIn('Park')
    expect(within(step(7)).getByText('Done')).toBeTruthy()
    await click('Start a new run', step(7))
    expect(within(step(4)).queryByText('Done')).toBeNull()
    expect(within(step(2)).getByRole('button', { name: 'Open gripper' })).toBeTruthy()
  })

  it('a new capture session clears the previous result and decision', async () => {
    setState(HOLDING)
    a.computeArmCamera.mockResolvedValue(ok({ result_id: doneFx.id }))
    a.getArmCameraResult.mockResolvedValue(ok(doneFx as ArmCameraResult))
    a.declineArmCamera.mockResolvedValue(ok({}))
    m.park.mockResolvedValue(ok(seq({ phase: 'idle', session_id: SID, captured: 40 })))
    m.moveToBase.mockResolvedValue(ok(seq({ running: true, phase: 'waiting_board', session_id: 'calib-new' })))
    await renderFlow(false)
    await click('Compute', step(5))
    await waitFor(() => expect(within(step(6)).getByText('Plausible')).toBeTruthy())
    await click('Decline', step(6))
    await click('Park arm', step(7))
    await confirmIn('Park')
    await click('Open gripper', step(2))
    await confirmIn('Move')
    expect(within(step(5)).queryByText('Done')).toBeNull()
    expect(screen.queryByText('Declined')).toBeNull()
  })

  it('without a live session the fits of the newest session stay reachable', async () => {
    setState(seq({}))
    await renderFlow(false)
    expect(within(step(5)).getByText('Finish step 2 first.')).toBeTruthy()
    expect(screen.getByRole('button', { name: 'More fits on this session' })).toBeTruthy()
    await click('More fits on this session')
    expect((screen.getByRole('button', { name: 'Compute arm offsets' }) as HTMLButtonElement).disabled).toBe(false)
  })

  it('one failed result poll does not lose the result', async () => {
    setState(HOLDING)
    a.computeArmCamera.mockResolvedValue(ok({ result_id: doneFx.id }))
    a.getArmCameraResult.mockResolvedValueOnce({ ok: false, status: 503, message: 'calibration node unreachable' }).mockResolvedValue(ok(doneFx as ArmCameraResult))
    vi.useFakeTimers({ shouldAdvanceTime: true })
    await renderFlow(false)
    await click('Compute', step(5))
    await act(async () => {
      await vi.advanceTimersByTimeAsync(RESULT_POLL_MS * 2)
    })
    expect(within(step(6)).getByText('Plausible')).toBeTruthy()
  })
})
