// @vitest-environment jsdom
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { act, cleanup, fireEvent, render, screen, within } from '@testing-library/react'
import { HeadArmCalibration, ARM_POLL_MS } from './HeadArmCalibration'
import stateFx from '../../../../head_rgbd_calibration/tests/fixtures/api/head_arm_state.json'
import resultFx from '../../../../head_rgbd_calibration/tests/fixtures/api/head_arm_result.json'
import applyFx from '../../../../head_rgbd_calibration/tests/fixtures/api/floor_apply.json'
import * as api from './headArmApi'
import type { HeadArmSequence } from './headArmApi'

vi.mock('./headArmApi')
const m = vi.mocked(api)

const seq = (over: Partial<HeadArmSequence>): HeadArmSequence => ({
  ...stateFx.sequence,
  running: false,
  phase: 'idle',
  status: 'idle',
  ...over,
})
const ok = <T,>(value: T) => ({ ok: true as const, value })
const SESSION = { id: 'calib-20261022-140000', created: 'x', views: 80, arm_poses: 24, results: [] }

function setState(s: HeadArmSequence, sessions = [SESSION]): void {
  m.getHeadArmState.mockResolvedValue(ok({ sequence: s, applied: null }))
  m.listHeadArmSessions.mockResolvedValue(ok(sessions))
}

beforeEach(() => {
  vi.resetAllMocks()
})
afterEach(() => {
  cleanup()
  vi.useRealTimers()
})

const click = async (name: string | RegExp) => {
  await act(async () => {
    fireEvent.click(screen.getByRole('button', { name }))
  })
}

describe('HeadArmCalibration', () => {
  it('moves only after the dialog confirm and names the joints', async () => {
    setState(seq({}))
    m.moveToBase.mockResolvedValue(ok(seq({ running: true, phase: 'moving_to_base' })))
    render(<HeadArmCalibration />)
    await screen.findByText(/without the board/)
    await click('Move to base pose')
    expect(m.moveToBase).not.toHaveBeenCalled()
    const dlg = within(screen.getByRole('dialog'))
    for (const j of ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper'])
      expect(dlg.getByText(new RegExp(j))).toBeTruthy()
    await click('Cancel')
    expect(m.moveToBase).not.toHaveBeenCalled()
    await screen.findByRole('button', { name: 'Move to base pose' })
    await click('Move to base pose')
    await click('Move')
    expect(m.moveToBase).toHaveBeenCalledWith(true)
  })

  it('idle state with null tier and arm_pose shows no null text', async () => {
    setState(seq({ tier: null, arm_pose: null, skipped: {}, session_id: 'x' }))
    render(<HeadArmCalibration />)
    await screen.findByText(/Tier -, pose - of/)
    expect(screen.queryByText(/null/)).toBeNull()
  })

  it('Board attached posts', async () => {
    setState(seq({ phase: 'waiting_board', running: true }))
    m.boardAttached.mockResolvedValue(ok(seq({ running: true, phase: 'capturing' })))
    render(<HeadArmCalibration />)
    await screen.findByText(/Clip the ChArUco board to the fixed jaw/)
    await click('Board attached')
    expect(m.boardAttached).toHaveBeenCalled()
  })

  it('shows tier progress and Stop while running', async () => {
    setState(seq({ running: true, phase: 'capturing', status: 'capturing tier 1', skipped: { unreachable: 3, collision: 2 } }))
    m.stopSequence.mockResolvedValue(ok(seq({ phase: 'holding' })))
    render(<HeadArmCalibration />)
    await screen.findByText(/Tier 1, pose 8 of 24, 30 captured, 5 skipped/)
    expect(screen.getByText(/capturing tier 1/)).toBeTruthy()
    await click('Stop')
    expect(m.stopSequence).toHaveBeenCalled()
  })

  it('big-joint dialog Allow and Skip', async () => {
    setState(seq({ running: true, phase: 'waiting_big_joints' }))
    m.allowBigJoints.mockResolvedValue(ok(seq({ running: true })))
    render(<HeadArmCalibration />)
    const dlg = await screen.findByRole('dialog')
    expect(within(dlg).getByText('Allow elbow and shoulder moves')).toBeTruthy()
    expect(within(dlg).getByText(/elbow_flex, shoulder_lift, shoulder_pan/)).toBeTruthy()
    expect(m.allowBigJoints).not.toHaveBeenCalled()
    await click('Allow')
    expect(m.allowBigJoints).toHaveBeenLastCalledWith(true)
    await click('Skip')
    expect(m.allowBigJoints).toHaveBeenLastCalledWith(false)
  })

  it('Park only after its confirm', async () => {
    setState(seq({ phase: 'holding', running: true }))
    m.park.mockResolvedValue(ok(seq({ phase: 'parking' })))
    render(<HeadArmCalibration />)
    await screen.findByText(/Remove the board from the gripper/)
    await click('Park arm')
    expect(m.park).not.toHaveBeenCalled()
    expect(screen.getByText('The board is removed')).toBeTruthy()
    await click('Park')
    expect(m.park).toHaveBeenCalledWith(true)
  })

  it('Compute shows the result table and Apply posts', async () => {
    setState(seq({}))
    m.computeHeadArm.mockResolvedValue(ok(resultFx as api.HeadArmResult))
    m.applyHeadArm.mockResolvedValue(ok({ ...applyFx, head_chain: { ...applyFx.head_chain, source: 'head_arm' } } as never))
    render(<HeadArmCalibration />)
    await screen.findByText(/Idle|without the board/)
    await click('Compute')
    expect(m.computeHeadArm).toHaveBeenCalledWith(SESSION.id)
    await screen.findByText('Passed')
    expect(screen.getByText('Mount x (mm)')).toBeTruthy()
    expect(screen.getByText('155.95')).toBeTruthy()
    expect(screen.getByText(/Held-out board error 0.48 mm/)).toBeTruthy()
    expect(screen.getByText('Mount yaw (deg)')).toBeTruthy()
    await click('Apply')
    expect(m.applyHeadArm).toHaveBeenCalledWith(resultFx.session, resultFx.id)
    await screen.findByText(/ros2-head_camera_cpp.service: restarted/)
    expect(screen.getByText(/source head_arm/)).toBeTruthy()
  })

  it('shows the pan axis parameters and the RMS per head offset', async () => {
    setState(seq({}))
    m.computeHeadArm.mockResolvedValue(ok(resultFx as api.HeadArmResult))
    render(<HeadArmCalibration />)
    await screen.findByText(/without the board/)
    await click('Compute')
    await screen.findByText('Passed')
    // Start 0, fitted and delta in mm, sigma, no floor value.
    const row = (label: string) => screen.getByText(label).closest('tr')?.textContent
    expect(row('Pan axis x (mm)')).toBe('Pan axis x (mm)0.00-30.10-30.100.40--')
    expect(row('Pan axis y (mm)')).toBe('Pan axis y (mm)0.00-9.90-9.900.60--')
    expect(screen.getByText('Head offset 0: 24 views, RMS 0.41 px')).toBeTruthy()
    expect(screen.getByText('Head offset 1: 24 views, RMS 0.43 px')).toBeTruthy()
  })

  it('a result without pan axis or per-offset RMS shows neither', async () => {
    setState(seq({}))
    const params = Object.fromEntries(Object.entries(resultFx.params).filter(([k]) => !k.startsWith('pan_axis')))
    const { offset_rms_px: _omit, ...older } = resultFx
    m.computeHeadArm.mockResolvedValue(ok({ ...older, params } as unknown as api.HeadArmResult))
    render(<HeadArmCalibration />)
    await screen.findByText(/without the board/)
    await click('Compute')
    await screen.findByText('Passed')
    expect(screen.queryByText('Pan axis x (mm)')).toBeNull()
    expect(screen.queryByText(/Head offset 0/)).toBeNull()
  })

  it('Compute is disabled without views', async () => {
    setState(seq({}), [{ ...SESSION, views: 0 }])
    render(<HeadArmCalibration />)
    await screen.findByText(/without the board/)
    expect(screen.getByRole('button', { name: 'Compute' }).hasAttribute('disabled')).toBe(true)
  })

  it('Apply is disabled for a failed result and the limits show', async () => {
    setState(seq({}))
    m.computeHeadArm.mockResolvedValue(
      ok({ ...resultFx, passed: false, failed_limits: ['holdout_board_mm 3 > 1'] } as api.HeadArmResult),
    )
    render(<HeadArmCalibration />)
    await screen.findByText(/without the board/)
    await click('Compute')
    await screen.findByText('Failed')
    expect(screen.getByText('holdout_board_mm 3 > 1')).toBeTruthy()
    expect(screen.getByRole('button', { name: 'Apply' }).hasAttribute('disabled')).toBe(true)
  })

  it('shows node errors in an Alert', async () => {
    setState(seq({}))
    m.moveToBase.mockResolvedValue({ ok: false, status: 409, message: 'arm busy' })
    render(<HeadArmCalibration />)
    await screen.findByText(/without the board/)
    await click('Move to base pose')
    await click('Move')
    expect((await screen.findByRole('alert')).textContent).toContain('arm busy')
  })

  it('polls state while a sequence runs', async () => {
    vi.useFakeTimers()
    setState(seq({ running: true, phase: 'capturing' }))
    render(<HeadArmCalibration />)
    await act(async () => {})
    const n = m.getHeadArmState.mock.calls.length
    await act(async () => {
      await vi.advanceTimersByTimeAsync(ARM_POLL_MS * 2)
    })
    expect(m.getHeadArmState.mock.calls.length).toBeGreaterThan(n)
  })
})
