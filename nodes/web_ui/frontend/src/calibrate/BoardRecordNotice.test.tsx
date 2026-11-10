// @vitest-environment jsdom
import { afterEach, describe, expect, it, vi } from 'vitest'
import { act, cleanup, fireEvent, render, screen, waitFor, within } from '@testing-library/react'
import { BoardRecordNotice, CLEAR_BODY, CLEAR_BUTTON, CLEAR_TITLE, boardRecordText, formatAge } from './BoardRecordNotice'
import * as api from './headArmApi'
import type { BoardRecord, HeadArmSequence } from './headArmApi'
import stateFx from '../../../../head_rgbd_calibration/tests/fixtures/api/head_arm_state.json'

/** The leftover record of the audit: no session, clipped 2026-11-09 18:44 UTC, not offered (too old). */
const OLD: BoardRecord = {
  file: '/var/lib/ros2/head_rgbd_calibration/head_arm_board.json',
  session_id: null,
  clipped_at: '2026-11-09T18:44:15+00:00',
  age_s: 39600,
  offered: false,
  problem: 'too old',
}
const CLEARED = { ...stateFx.sequence, board_record: null } as HeadArmSequence
const PARK_HINT = 'park it from the flow.'

afterEach(() => {
  cleanup()
  vi.restoreAllMocks()
})

const button = () => screen.getByRole('button', { name: CLEAR_BUTTON }) as HTMLButtonElement

describe('BoardRecordNotice', () => {
  it('formats an age in the largest fitting unit', () => {
    expect(formatAge(45)).toBe('45 s')
    expect(formatAge(600)).toBe('10 min')
    expect(formatAge(39600)).toBe('11 h')
    expect(formatAge(3 * 86400)).toBe('3 d')
  })

  it('names the session, the attach time and the age, or why the record is unreadable', () => {
    expect(boardRecordText(OLD)).toBe(
      'The node records the calibration board on the gripper: attached without a capture session at 2026-11-09T18:44:15+00:00, 11 h ago.',
    )
    expect(boardRecordText({ ...OLD, session_id: 'calib-1', age_s: null })).toContain('attached in session calib-1 at')
    const unreadable = { ...OLD, clipped_at: null, age_s: null, problem: 'unreadable: bad json' }
    expect(boardRecordText(unreadable)).toBe('The node has a board record it cannot read (unreadable: bad json).')
  })

  it('renders nothing without a record', () => {
    const { container } = render(<BoardRecordNotice record={null} running={false} parkHint={PARK_HINT} />)
    expect(container.textContent).toBe('')
  })

  it('shows an old record with both ways out and keeps the clear behind a confirm', async () => {
    const clear = vi.spyOn(api, 'clearBoardRecord').mockResolvedValue({ ok: true, value: CLEARED })
    const onCleared = vi.fn()
    render(<BoardRecordNotice record={OLD} running={false} parkHint={PARK_HINT} onCleared={onCleared} />)
    expect(screen.getByText(/Home arm is refused while the record exists\. Board still on the gripper: park it from the flow\. Board already off: clear the record\./)).toBeTruthy()
    fireEvent.click(button())
    const dialog = await screen.findByRole('dialog')
    expect(within(dialog).getByText(CLEAR_TITLE)).toBeTruthy()
    expect(within(dialog).getByText(CLEAR_BODY)).toBeTruthy()
    expect(clear).not.toHaveBeenCalled()
    await act(async () => {
      fireEvent.click(within(dialog).getByRole('button', { name: 'Clear the record' }))
    })
    expect(clear).toHaveBeenCalledTimes(1)
    expect(onCleared).toHaveBeenCalledWith(CLEARED)
  })

  it('cancel clears nothing', async () => {
    const clear = vi.spyOn(api, 'clearBoardRecord')
    render(<BoardRecordNotice record={OLD} running={false} parkHint={PARK_HINT} />)
    fireEvent.click(button())
    fireEvent.click(within(await screen.findByRole('dialog')).getByRole('button', { name: 'Cancel' }))
    await waitFor(() => expect(screen.queryByRole('dialog')).toBeNull())
    expect(clear).not.toHaveBeenCalled()
  })

  it('the clear is off while a sequence or park runs', () => {
    render(<BoardRecordNotice record={OLD} running parkHint={PARK_HINT} />)
    expect(button().disabled).toBe(true)
  })

  it('shows the node refusal', async () => {
    vi.spyOn(api, 'clearBoardRecord').mockResolvedValue({ ok: false, status: 409, message: 'an arm to camera compute is running' })
    const onCleared = vi.fn()
    render(<BoardRecordNotice record={OLD} running={false} parkHint={PARK_HINT} onCleared={onCleared} />)
    fireEvent.click(button())
    await act(async () => {
      fireEvent.click(within(await screen.findByRole('dialog')).getByRole('button', { name: 'Clear the record' }))
    })
    expect(screen.getByText('an arm to camera compute is running')).toBeTruthy()
    expect(onCleared).not.toHaveBeenCalled()
  })
})
