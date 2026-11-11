// @vitest-environment jsdom
import { afterEach, describe, expect, it, vi } from 'vitest'
import { cleanup, fireEvent, render, screen, waitFor } from '@testing-library/react'
import { ServoCalibrationCard } from './ServoCalibrationCard'
import type { CalibrationCapture, CalibrationPlan, CalibrationPlanJoint, ServoCalibrationState, ServoGroup } from './servoApi'

afterEach(cleanup)

function planJoint(over: Partial<CalibrationPlanJoint> = {}): CalibrationPlanJoint {
  return {
    name: 'head_pan', id: 1, inverted: false, home_steps: 2100, target_steps: 2048, home_rad: 0.08, target_rad: 0,
    offset_before_steps: 0, offset_change_steps: 52, change_deg: 4.6, new_offset_steps: 52,
    limits_before: [0, 4095], limits_after: [300, 3700], range_deg: 299, full_turn: false,
    exceeds_max_change: false, narrower_than_required: false, problems: [], ...over,
  }
}

function plan(over: Partial<CalibrationPlan> = {}, joint: Partial<CalibrationPlanJoint> = {}): CalibrationPlan {
  return { run_id: 'p1', created_at: 'now', needs_confirm: false, ok: true, joints: { head_pan: planJoint(joint) }, ...over }
}

function group(opts: { torque?: 0 | 1; home?: boolean; capture?: CalibrationCapture | null; plan?: CalibrationPlan | null } = {}): ServoGroup {
  const sc = {
    config: {} as ServoCalibrationState['config'],
    home: opts.home ? { recorded_at: 't', joints: { head_pan: { steps: 2100, rad: 0.08 } } } : null,
    capture: opts.capture ?? null,
    plan: opts.plan ?? null,
    last_result: null,
  }
  return {
    id: 'head', label: 'Head', neutral: false, age_s: 0.1, stale: false, calibration_stale: null,
    zero_restore: false, move_to_neutral: false, set_home: false, servo_calibration: true,
    home_pose_hint: 'pointing directly forward, perpendicular to ground',
    state: {
      namespace: 'head', released: (opts.torque ?? 0) === 0, joints: { head_pan: opts.torque ?? 0 }, stamp_ns: 1,
      neutral_enabled: false, neutral_capture: null, last_save: null, neutral_max_shift_deg: null, servo_calibration: sc,
    },
  }
}

const ok = () => vi.fn().mockResolvedValue({ ok: true, message: 'stored' })
const running: CalibrationCapture = {
  running: true, paused: false, samples: 5,
  joints: { head_pan: { present_steps: 2000, min_steps: 1500, max_steps: 2500, range_deg: 88 } },
}

describe('ServoCalibrationCard', () => {
  it('shows the home pose hint at Record home', () => {
    render(<ServoCalibrationCard group={group()} onAction={ok()} />)
    expect(screen.getByText(/pointing directly forward, perpendicular to ground/)).toBeTruthy()
  })

  it('record home disabled until torque is off', () => {
    const { rerender } = render(<ServoCalibrationCard group={group({ torque: 1 })} onAction={ok()} />)
    const locked = screen.queryByRole('button', { name: 'Record home' }) as HTMLButtonElement | null
    expect(locked === null || locked.disabled).toBe(true)
    rerender(<ServoCalibrationCard group={group({ torque: 0 })} onAction={ok()} />)
    expect((screen.getByRole('button', { name: 'Record home' }) as HTMLButtonElement).disabled).toBe(false)
  })

  it('range bars show min max and present', () => {
    render(<ServoCalibrationCard group={group({ home: true, capture: running })} onAction={ok()} />)
    expect(screen.getByText(/min 1500 max 2500 present 2000 range 88 deg/)).toBeTruthy()
  })

  it('store disabled while a capture runs', () => {
    render(<ServoCalibrationCard group={group({ home: true, capture: running, plan: plan() })} onAction={ok()} />)
    expect((screen.getByRole('button', { name: 'Store' }) as HTMLButtonElement).disabled).toBe(true)
  })

  it('store shows planned offsets and limits and stores bound to the plan', async () => {
    const onAction = ok()
    render(<ServoCalibrationCard group={group({ home: true, plan: plan() })} onAction={onAction} />)
    expect(screen.getByText('Return the joints to home and let go')).toBeTruthy()
    expect(screen.getByText('+4.6')).toBeTruthy()
    expect(screen.getByText('52')).toBeTruthy()
    expect(screen.getByText('0..4095 -> 300..3700')).toBeTruthy()
    fireEvent.click(screen.getByRole('button', { name: 'Store' }))
    await waitFor(() => expect(onAction).toHaveBeenCalledWith('calibration_store', 'p1'))
    expect(await screen.findByText('stored')).toBeTruthy()
  })

  it('large change asks for confirm and calls store_confirmed', async () => {
    const onAction = ok()
    const p = plan({ needs_confirm: true }, { exceeds_max_change: true, change_deg: 40, narrower_than_required: true })
    render(<ServoCalibrationCard group={group({ home: true, plan: p })} onAction={onAction} />)
    fireEvent.click(screen.getByRole('button', { name: 'Store' }))
    expect(onAction).not.toHaveBeenCalled()
    expect(screen.getByText(/head_pan: large change \+40.0 deg/)).toBeTruthy()
    expect(screen.getByText(/head_pan: narrower travel than required/)).toBeTruthy()
    fireEvent.click(screen.getByRole('button', { name: 'Store anyway' }))
    await waitFor(() => expect(onAction).toHaveBeenCalledWith('calibration_store_confirmed', 'p1'))
  })

  it('plan problems disable Store', () => {
    const p = plan({ ok: false }, { problems: ['range below minimum'] })
    render(<ServoCalibrationCard group={group({ home: true, plan: p })} onAction={ok()} />)
    expect(screen.getByText('range below minimum')).toBeTruthy()
    expect((screen.getByRole('button', { name: 'Store' }) as HTMLButtonElement).disabled).toBe(true)
  })

  it('shows home not set message', async () => {
    const onAction = vi.fn().mockResolvedValue({ ok: true, message: 'stored, set_home refused: no readings', home_set: false })
    render(<ServoCalibrationCard group={group({ home: true, plan: plan() })} onAction={onAction} />)
    fireEvent.click(screen.getByRole('button', { name: 'Store' }))
    expect(await screen.findByText('Arm home not set')).toBeTruthy()
    expect(screen.getByText(/set_home refused: no readings/)).toBeTruthy()
  })

  it('the store outcome is cleared by a new plan and by a non-store action', async () => {
    const onAction = ok()
    const view = render(<ServoCalibrationCard group={group({ home: true, plan: plan() })} onAction={onAction} />)
    fireEvent.click(screen.getByRole('button', { name: 'Store' }))
    expect(await screen.findByText('stored')).toBeTruthy()
    view.rerender(<ServoCalibrationCard group={group({ home: true, plan: plan({ run_id: 'p2' }) })} onAction={onAction} />)
    await waitFor(() => expect(screen.queryByText('stored')).toBeNull())
    fireEvent.click(screen.getByRole('button', { name: 'Store' }))
    expect(await screen.findByText('stored')).toBeTruthy()
    fireEvent.click(screen.getByRole('button', { name: 'Re-enable torque' }))
    await waitFor(() => expect(screen.queryByText('stored')).toBeNull())
  })

  it('the store outcome stays when the bridge drops the plan after a store, and a later new plan clears it', async () => {
    const onAction = ok()
    const view = render(<ServoCalibrationCard group={group({ home: true, plan: plan() })} onAction={onAction} />)
    fireEvent.click(screen.getByRole('button', { name: 'Store' }))
    expect(await screen.findByText('stored')).toBeTruthy()
    view.rerender(<ServoCalibrationCard group={group({ home: true, plan: null })} onAction={onAction} />)
    await new Promise((r) => setTimeout(r, 50))
    expect(screen.getByText('stored')).toBeTruthy()
    view.rerender(<ServoCalibrationCard group={group({ home: true, plan: plan({ run_id: 'p2' }) })} onAction={onAction} />)
    await waitFor(() => expect(screen.queryByText('stored')).toBeNull())
  })

  it('re-enable torque is visible while released and calls enable', async () => {
    const onAction = ok()
    render(<ServoCalibrationCard group={group()} onAction={onAction} />)
    fireEvent.click(screen.getByRole('button', { name: 'Re-enable torque' }))
    await waitFor(() => expect(onAction).toHaveBeenCalledWith('enable', undefined))
  })
})
