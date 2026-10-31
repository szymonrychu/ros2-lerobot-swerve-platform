// @vitest-environment jsdom
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { act, cleanup, fireEvent, render, screen, within } from '@testing-library/react'
import { ArmCalibration } from './ArmCalibration'
import { ARM_POLL_MS } from './HeadArmCalibration'
import stateFx from '../../../../head_rgbd_calibration/tests/fixtures/api/head_arm_state.json'
import applyFx from '../../../../head_rgbd_calibration/tests/fixtures/api/floor_apply.json'
import * as api from './headArmApi'
import type { ArmResult, HeadArmSequence } from './headArmApi'
import type { AppliedHeadChain } from './headFloorApi'

vi.mock('./headArmApi')
const m = vi.mocked(api)

const seq = (over: Partial<HeadArmSequence>): HeadArmSequence => ({
  ...stateFx.sequence,
  running: false,
  phase: 'idle',
  status: 'idle',
  skipped: {},
  ...over,
})
const ok = <T,>(value: T) => ({ ok: true as const, value })
const SESSION = { id: 'calib-20261031-110000', created: 'x', views: 60, arm_poses: 20, results: [] as string[] }
const APPLIED = stateFx.applied as AppliedHeadChain
const DEG = Math.PI / 180

const RESULT: ArmResult = {
  id: 'r-20261031-120000',
  session: SESSION.id,
  kind: 'arm',
  floor_session: 'floor-20261030-090000',
  floor: { session: 'floor-20261030-090000', result: 'r-20261030-091000' },
  created: '2026-10-31T12:00:00+01:00',
  passed: true,
  failed_limits: [],
  error: null,
  rms_px: 0.61,
  holdout_rms_px: 0.72,
  holdout_tool_mm: 1.9,
  wrist_roll_fitted: false,
  tool_fitted: true,
  tool_offset_m: [0.001, -0.002, -0.0981],
  offsets: {
    shoulder_pan: { value: 2 * DEG, sigma: 0.1 * DEG, current: 1 * DEG },
    wrist_roll: { value: 0, sigma: null, current: 0 },
  },
  head: {
    mount_x_m: { value: 0.1565, sigma: 0.0001, urdf: 0.1555 },
    mount_y_m: { value: 0.0995, sigma: 0.0001, urdf: 0.1 },
    mount_yaw_deg: { value: 0.25, sigma: 0.01, urdf: 0.1 },
  },
  limits: [{ name: 'rms_px', value: 0.61, limit: 1.5, ok: true }],
  snippet: '    joint_offsets_rad:\n      shoulder_pan: 0.0349\n',
}

function setState(s: HeadArmSequence, applied: AppliedHeadChain | null = APPLIED, sessions = [SESSION]): void {
  m.getHeadArmState.mockResolvedValue(ok({ sequence: s, applied }))
  m.listHeadArmSessions.mockResolvedValue(ok(sessions))
}

const click = async (name: string | RegExp) => {
  await act(async () => {
    fireEvent.click(screen.getByRole('button', { name }))
  })
}

/** Renders the section and opens it with "Arm calibrate". */
async function open(): Promise<void> {
  render(<ArmCalibration />)
  await act(async () => {})
  await click('Arm calibrate')
}

beforeEach(() => {
  vi.resetAllMocks()
})
afterEach(() => {
  cleanup()
  vi.useRealTimers()
})

describe('ArmCalibration', () => {
  it('disabled without an applied head chain', async () => {
    setState(seq({}), null)
    render(<ArmCalibration />)
    await screen.findByText(/apply a head calibration first/)
    expect(screen.getByRole('button', { name: 'Arm calibrate' }).hasAttribute('disabled')).toBe(true)
    expect(screen.queryByRole('button', { name: 'Start' })).toBeNull()
    cleanup()
    setState(seq({}))
    await open()
    expect(screen.queryByText(/apply a head calibration first/)).toBeNull()
    expect(screen.getByRole('button', { name: 'Start' })).toBeTruthy()
  })

  it('start runs wrist and elbow tiers', async () => {
    vi.useFakeTimers()
    setState(seq({}))
    m.moveToBase.mockResolvedValue(ok(seq({ running: true, phase: 'moving_to_base' })))
    m.boardAttached.mockResolvedValue(ok(seq({ running: true, phase: 'capturing', tier: 0 })))
    await open()
    expect(screen.getByText(/board off the floor, clipped to the fixed jaw/)).toBeTruthy()
    await click('Start')
    expect(screen.queryByRole('dialog')).toBeNull()
    expect(m.moveToBase).toHaveBeenCalledWith(true)
    expect(m.boardAttached).not.toHaveBeenCalled()
    setState(seq({ running: true, phase: 'waiting_board' }))
    await act(async () => {
      await vi.advanceTimersByTimeAsync(ARM_POLL_MS)
    })
    expect(m.boardAttached).toHaveBeenCalledTimes(1)
    await act(async () => {
      await vi.advanceTimersByTimeAsync(ARM_POLL_MS * 2)
    })
    expect(m.boardAttached).toHaveBeenCalledTimes(1)
    expect(m.allowBigJoints).not.toHaveBeenCalled()
    expect(screen.queryByRole('dialog')).toBeNull()
  })

  it('start attaches the board at once when the base move already waits for it', async () => {
    setState(seq({}))
    m.moveToBase.mockResolvedValue(ok(seq({ running: true, phase: 'waiting_board' })))
    m.boardAttached.mockResolvedValue(ok(seq({ running: true, phase: 'capturing', tier: 0 })))
    await open()
    await click('Start')
    expect(m.boardAttached).toHaveBeenCalledTimes(1)
    expect(m.allowBigJoints).not.toHaveBeenCalled()
  })

  it('allow big joints only after the shoulder confirm', async () => {
    setState(seq({ running: true, phase: 'waiting_big_joints' }))
    m.allowBigJoints.mockResolvedValue(ok(seq({ running: true, phase: 'capturing', tier: 2 })))
    await open()
    expect(screen.queryByRole('dialog')).toBeNull()
    await click('Allow shoulder moves')
    const dlg = within(screen.getByRole('dialog'))
    expect(dlg.getByText(/shoulder_lift/)).toBeTruthy()
    expect(dlg.getByText(/shoulder_pan/)).toBeTruthy()
    expect(m.allowBigJoints).not.toHaveBeenCalled()
    await click('Cancel')
    expect(m.allowBigJoints).not.toHaveBeenCalled()
    await screen.findByRole('button', { name: 'Allow shoulder moves' })
    await click('Allow shoulder moves')
    await click('Confirm')
    expect(m.allowBigJoints).toHaveBeenCalledTimes(1)
    expect(m.allowBigJoints).toHaveBeenCalledWith(true)
  })

  it('Allow shoulder moves is disabled until the sequence waits for it', async () => {
    setState(seq({ running: true, phase: 'capturing', tier: 1 }))
    await open()
    expect(screen.getByRole('button', { name: 'Allow shoulder moves' }).hasAttribute('disabled')).toBe(true)
  })

  it('stop calls the route and the progress names the skip reasons', async () => {
    setState(
      seq({
        running: true,
        phase: 'capturing',
        tier: 1,
        arm_pose: 4,
        total_poses: 12,
        captured: 9,
        status: 'capturing tier 1',
        skipped: { 'inside D435 min-Z': 3, 'outside colour FOV': 2 },
      }),
    )
    m.stopSequence.mockResolvedValue(ok(seq({ phase: 'holding' })))
    await open()
    expect(screen.getByText(/Tier 1, step 5 of 12, 9 captured, 5 skipped - capturing tier 1/)).toBeTruthy()
    expect(screen.getByText('inside D435 min-Z: 3')).toBeTruthy()
    expect(screen.getByText('outside colour FOV: 2')).toBeTruthy()
    await click('Stop')
    expect(m.stopSequence).toHaveBeenCalled()
  })

  it('compute shows the result card', async () => {
    setState(seq({}))
    m.computeArm.mockResolvedValue(ok(RESULT))
    await open()
    await click('Compute')
    expect(m.computeArm).toHaveBeenCalledWith(SESSION.id)
    await screen.findByText('Passed')
    const row = (label: string) => screen.getByText(label).closest('tr')?.textContent
    expect(row('shoulder_pan')).toBe('shoulder_pan1.0002.000+1.0000.100')
    expect(row('wrist_roll')).toBe('wrist_roll0.0000.000+0.000-')
    expect(screen.getByText(/wrist_roll fixed/)).toBeTruthy()
    expect(screen.getByText(/Tool point new \(mm\) 1.0, -2.0, -98.1 \(fitted\), held-out error 1.90 mm/)).toBeTruthy()
    expect(screen.getByText(/Head change: x \+1.00 mm, y -0.50 mm, yaw \+0.150 deg/)).toBeTruthy()
    expect(screen.getByText(/shoulder_pan: 0.0349/)).toBeTruthy()
    const download = screen.getByRole('link', { name: 'Download zip' })
    expect(download.getAttribute('href')).toBe(`/api/calibration/rgbd/sessions/${SESSION.id}/results/${RESULT.id}/download`)
    expect(screen.getByText(/offsets are committed by hand/)).toBeTruthy()
  })

  it('a kept tool point says so', async () => {
    setState(seq({}))
    m.computeArm.mockResolvedValue(ok({ ...RESULT, tool_fitted: false, wrist_roll_fitted: true }))
    await open()
    await click('Compute')
    await screen.findByText('Passed')
    expect(screen.getByText(/Tool point current \(mm\) 1.0, -2.0, -98.1 \(kept\)/)).toBeTruthy()
    expect(screen.getByText(/wrist_roll fitted/)).toBeTruthy()
  })

  it('copy writes the snippet', async () => {
    const writeText = vi.fn(async () => undefined)
    Object.defineProperty(navigator, 'clipboard', { value: { writeText }, configurable: true })
    setState(seq({}))
    m.computeArm.mockResolvedValue(ok(RESULT))
    await open()
    await click('Compute')
    await screen.findByText('Passed')
    await click('Copy')
    expect(writeText).toHaveBeenCalledWith(RESULT.snippet)
  })

  it('apply disabled on a failed result', async () => {
    setState(seq({}))
    m.computeArm.mockResolvedValue(ok({ ...RESULT, passed: false, failed_limits: ['holdout_tool_mm 9 > 5'] }))
    await open()
    await click('Compute')
    await screen.findByText('Failed')
    expect(screen.getByText('holdout_tool_mm 9 > 5')).toBeTruthy()
    expect(screen.getByRole('button', { name: 'Apply head part' }).hasAttribute('disabled')).toBe(true)
  })

  it('an input error shows the error and no table', async () => {
    setState(seq({}))
    const { rms_px: _r, offsets: _o, head: _h, snippet: _s, ...base } = RESULT
    m.computeArm.mockResolvedValue(ok({ ...base, passed: false, error: 'plate_mount is not set' }))
    await open()
    await click('Compute')
    await screen.findByText('Failed')
    expect(screen.getByText('plate_mount is not set')).toBeTruthy()
    expect(screen.queryByRole('table')).toBeNull()
    expect(screen.queryByRole('button', { name: 'Copy' })).toBeNull()
  })

  it('Apply head part posts a passed result and shows the restart state', async () => {
    setState(seq({}))
    m.computeArm.mockResolvedValue(ok(RESULT))
    m.applyHeadArm.mockResolvedValue(ok({ ...applyFx, head_chain: { ...applyFx.head_chain, source: 'arm' } } as never))
    await open()
    await click('Compute')
    await screen.findByText('Passed')
    await click('Apply head part')
    expect(m.applyHeadArm).toHaveBeenCalledWith(RESULT.session, RESULT.id)
    await screen.findByText(/source arm/)
  })

  it('shows the latest arm result of the session through getHeadArmResult', async () => {
    setState(seq({}), APPLIED, [{ ...SESSION, results: ['r-old', RESULT.id] }])
    m.getHeadArmResult.mockResolvedValue(ok(RESULT))
    await open()
    await screen.findByText('Passed')
    expect(m.getHeadArmResult).toHaveBeenCalledWith(SESSION.id, RESULT.id)
  })
})
