// @vitest-environment jsdom
import { afterEach, describe, expect, it, vi } from 'vitest'
import { cleanup, fireEvent, render, screen, waitFor, within } from '@testing-library/react'
import { GeneralSection } from './GeneralSection'
import * as generalApi from './generalApi'
import * as headArmApi from './headArmApi'
import * as headFloorApi from './headFloorApi'
import type { HeadArmSequence } from './headArmApi'

const SEQ: HeadArmSequence = {
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

function stubSequence(seq: Partial<HeadArmSequence> = {}) {
  vi.spyOn(headFloorApi, 'getFloorState').mockResolvedValue({
    ok: true,
    value: { sweep: { running: false, session_id: null, pose: 0, total: 0, observations: 0, skipped: 0, status: 'idle', error: null }, applied: null },
  })
  return vi.spyOn(headArmApi, 'getHeadArmState').mockResolvedValue({ ok: true, value: { sequence: { ...SEQ, ...seq }, applied: null } })
}

afterEach(() => {
  cleanup()
  vi.restoreAllMocks()
})

const ALL = generalApi.GENERAL_ACTIONS.map((a) => a.id)

describe('GeneralSection', () => {
  it('shows Abort and the offered actions only', async () => {
    stubSequence()
    render(<GeneralSection actions={['home_arm', 'home_head']} rgbd />)
    expect(screen.getByRole('button', { name: 'Abort calibration' })).toBeTruthy()
    expect(screen.getByRole('button', { name: 'Home arm' })).toBeTruthy()
    expect(screen.getByRole('button', { name: 'Home camera' })).toBeTruthy()
    expect(screen.queryByRole('button', { name: 'Open gripper' })).toBeNull()
  })

  it('every moving action asks for a confirm, then runs and shows the answer', async () => {
    stubSequence()
    const run = vi.spyOn(generalApi, 'runGeneralAction').mockResolvedValue({ ok: true, value: { ok: true, message: 'converged: home' } })
    render(<GeneralSection actions={ALL} rgbd />)
    for (const action of generalApi.GENERAL_ACTIONS) {
      run.mockClear()
      fireEvent.click(screen.getByRole('button', { name: action.label }))
      const dialog = await screen.findByRole('dialog')
      expect(within(dialog).getByText(action.confirm)).toBeTruthy()
      expect(run).not.toHaveBeenCalled()
      fireEvent.click(within(dialog).getByRole('button', { name: action.label }))
      await waitFor(() => expect(run).toHaveBeenCalledWith(action.id))
      await waitFor(() => expect(screen.queryByRole('dialog')).toBeNull())
    }
    expect(await screen.findByText('Home steering servos: converged: home')).toBeTruthy()
  })

  it('cancel runs nothing', async () => {
    stubSequence()
    const run = vi.spyOn(generalApi, 'runGeneralAction')
    render(<GeneralSection actions={ALL} rgbd />)
    fireEvent.click(screen.getByRole('button', { name: 'Close gripper' }))
    fireEvent.click(within(await screen.findByRole('dialog')).getByRole('button', { name: 'Cancel' }))
    await waitFor(() => expect(screen.queryByRole('dialog')).toBeNull())
    expect(run).not.toHaveBeenCalled()
  })

  it('a refusal shows the backend message as an error', async () => {
    stubSequence()
    vi.spyOn(generalApi, 'runGeneralAction').mockResolvedValue({ ok: false, status: 409, message: 'Arm servo torque is off' })
    render(<GeneralSection actions={ALL} rgbd />)
    fireEvent.click(screen.getByRole('button', { name: 'Open gripper' }))
    fireEvent.click(within(await screen.findByRole('dialog')).getByRole('button', { name: 'Open gripper' }))
    expect(await screen.findByText('Open gripper: Arm servo torque is off')).toBeTruthy()
  })

  it('Abort confirms, stops the running work and lists what stopped', async () => {
    stubSequence()
    const abort = vi.spyOn(generalApi, 'abortCalibration').mockResolvedValue({ stopped: ['arm-camera capture'], errors: [] })
    render(<GeneralSection actions={ALL} rgbd />)
    fireEvent.click(screen.getByRole('button', { name: 'Abort calibration' }))
    const dialog = await screen.findByRole('dialog')
    expect(within(dialog).getByText(/holds where it is/)).toBeTruthy()
    fireEvent.click(within(dialog).getByRole('button', { name: 'Abort' }))
    await waitFor(() => expect(abort).toHaveBeenCalled())
    expect(await screen.findByText('Stopped: arm-camera capture. The arm holds where it is.')).toBeTruthy()
  })

  it('Abort with nothing running says so and shows the errors', async () => {
    stubSequence()
    vi.spyOn(generalApi, 'abortCalibration').mockResolvedValue({ stopped: [], errors: ['floor sweep: down'] })
    render(<GeneralSection actions={ALL} rgbd />)
    fireEvent.click(screen.getByRole('button', { name: 'Abort calibration' }))
    fireEvent.click(within(await screen.findByRole('dialog')).getByRole('button', { name: 'Abort' }))
    expect(await screen.findByText('Nothing was running.')).toBeTruthy()
    expect(screen.getByText('floor sweep: down')).toBeTruthy()
  })

  it('arm and head moves are off while a calibration sequence is live; steering stays on', async () => {
    stubSequence({ running: true, phase: 'tier' })
    render(<GeneralSection actions={ALL} rgbd />)
    await waitFor(() => expect((screen.getByRole('button', { name: 'Home arm' }) as HTMLButtonElement).disabled).toBe(true))
    expect((screen.getByRole('button', { name: 'Open gripper' }) as HTMLButtonElement).disabled).toBe(true)
    expect((screen.getByRole('button', { name: 'Close gripper' }) as HTMLButtonElement).disabled).toBe(true)
    expect((screen.getByRole('button', { name: 'Home camera' }) as HTMLButtonElement).disabled).toBe(true)
    expect((screen.getByRole('button', { name: 'Home steering servos' }) as HTMLButtonElement).disabled).toBe(false)
    expect(screen.getByText(/Abort the running calibration first/)).toBeTruthy()
  })

  it('arm moves are off while the arm holds the board or parks', async () => {
    for (const phase of ['holding', 'parking']) {
      stubSequence({ running: phase === 'parking', phase })
      render(<GeneralSection actions={ALL} rgbd />)
      await waitFor(() => expect((screen.getByRole('button', { name: 'Open gripper' }) as HTMLButtonElement).disabled).toBe(true))
      expect((screen.getByRole('button', { name: 'Home arm' }) as HTMLButtonElement).disabled).toBe(true)
      cleanup()
      vi.restoreAllMocks()
    }
  })

  it('Home camera is off while a floor sweep moves the head', async () => {
    stubSequence()
    vi.spyOn(headFloorApi, 'getFloorState').mockResolvedValue({
      ok: true,
      value: { sweep: { running: true, session_id: 's', pose: 1, total: 9, observations: 3, skipped: 0, status: 'sweeping', error: null }, applied: null },
    })
    render(<GeneralSection actions={ALL} rgbd />)
    await waitFor(() => expect((screen.getByRole('button', { name: 'Home camera' }) as HTMLButtonElement).disabled).toBe(true))
    expect((screen.getByRole('button', { name: 'Home arm' }) as HTMLButtonElement).disabled).toBe(false)
  })

  it('without the RGB-D node it does not read the sequence', async () => {
    const spy = stubSequence()
    render(<GeneralSection actions={ALL} rgbd={false} />)
    await new Promise((r) => setTimeout(r, 20))
    expect(spy).not.toHaveBeenCalled()
    expect((screen.getByRole('button', { name: 'Home arm' }) as HTMLButtonElement).disabled).toBe(false)
    expect(screen.queryByRole('button', { name: 'Abort calibration' })).toBeNull()
  })
})
