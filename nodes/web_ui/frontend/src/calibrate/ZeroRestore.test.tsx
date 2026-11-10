// @vitest-environment jsdom
import { afterEach, describe, expect, it, vi } from 'vitest'
import { cleanup, fireEvent, render, screen, waitFor, within } from '@testing-library/react'
import { ServoNeutral } from './ServoNeutral'
import type { ServoGroup, TorqueState, ZeroRestorePlanJoint, ZeroRestoreState } from './servoApi'

const DIALOG_LARGE = 'The arm rests folded on its rest and the large change is intended'
const MOVE_TEXT = 'The arm moves to the neutral pose: all joints at 0. Keep the area clear.'

function planJoint(over: Partial<ZeroRestorePlanJoint> = {}): ZeroRestorePlanJoint {
  return {
    present_steps: 1000,
    present_rad: -0.913,
    target_steps: 1922,
    target_rad: -1.542,
    offset_before_steps: -40,
    offset_change_steps: 901,
    change_deg: 79.2,
    new_offset_steps: 861,
    exceeds_max_change: true,
    limits: [0, 4095],
    ...over,
  }
}

function zero(over: Partial<ZeroRestoreState> = {}): ZeroRestoreState {
  return {
    max_change_deg: 15,
    reference: { source: 'seed', saved_at: null, joints: { shoulder_lift: -1.542, wrist_flex: 0.862 } },
    reference_error: null,
    stale_joint_offsets: false,
    plan: null,
    last_result: null,
    ...over,
  }
}

// A fresh plan: created now, so it is inside max_age_s (the panel disables Apply for an expired dry run).
function plan(needsConfirm: boolean, runId = 'r1', createdAt = new Date().toISOString()) {
  return {
    run_id: runId,
    created_at: createdAt,
    max_age_s: 60,
    reference_source: 'seed' as const,
    needs_confirm: needsConfirm,
    joints: {
      shoulder_lift: planJoint({ exceeds_max_change: needsConfirm }),
      wrist_flex: planJoint({ present_rad: 0.794, target_rad: 0.862, change_deg: 4.1, offset_change_steps: 47, exceeds_max_change: false }),
    },
  }
}

function state(released: boolean, zr: ZeroRestoreState | null): TorqueState {
  const v = released ? 0 : 1
  return {
    namespace: 'follower',
    released,
    joints: { shoulder_lift: v, wrist_flex: v },
    stamp_ns: 1,
    neutral_enabled: true,
    neutral_capture: null,
    last_save: null,
    neutral_max_shift_deg: 15,
    zero_restore: zr,
  }
}

function arm(released: boolean, zr: ZeroRestoreState | null, over: Partial<ServoGroup> = {}): ServoGroup {
  return {
    id: 'arm',
    label: 'Arm',
    neutral: true,
    state: state(released, zr),
    age_s: 0.1,
    stale: false,
    calibration_stale: null,
    zero_restore: true,
    move_to_neutral: true,
    set_home: true,
    ...over,
  }
}

let groups: ServoGroup[]
let posts: string[]

function mockFetch(): void {
  posts = []
  vi.stubGlobal(
    'fetch',
    vi.fn(async (url: string, init?: RequestInit) => {
      if (init?.method === 'POST') {
        posts.push(url)
        return new Response(JSON.stringify({ ok: true, message: 'bridge says ok' }), { status: 200 })
      }
      return new Response(JSON.stringify({ groups }), { status: 200 })
    }),
  )
}

afterEach(() => {
  cleanup()
  vi.unstubAllGlobals()
})

const panel = async () => screen.findByRole('region', { name: 'Arm zero restore' })
const button = (root: HTMLElement, name: string | RegExp) => within(root).getByRole('button', { name }) as HTMLButtonElement
const dialog = () => screen.getByRole('dialog')
const POST = (action: string) => `/api/calibration/servos/arm/${action}`
// Apply posts carry the run_id of the plan the user was shown (the backend refuses another plan).
const APPLY_POST = (action: string, runId = 'r1') => `${POST(action)}?plan_run_id=${runId}`

describe('Arm zero restore panel', () => {
  it('is absent without the zero_restore flag', async () => {
    groups = [arm(true, null, { zero_restore: false, move_to_neutral: false })]
    mockFetch()
    render(<ServoNeutral />)
    await screen.findByRole('region', { name: 'Arm' })
    expect(screen.queryByRole('region', { name: 'Arm zero restore' })).toBeNull()
    expect(screen.queryByRole('button', { name: 'Move to neutral pose' })).toBeNull()
  })

  it('torque on: only Torque off arm is enabled, with what to expect', async () => {
    groups = [arm(false, zero())]
    mockFetch()
    render(<ServoNeutral />)
    const p = await panel()
    expect(button(p, '1. Torque off arm').disabled).toBe(false)
    expect(button(p, '2. Dry run').disabled).toBe(true)
    expect(button(p, '3. Apply').disabled).toBe(true)
    expect(button(p, '4. Re-enable torque').disabled).toBe(true)
    expect(within(p).getByText('The arm goes limp; rest it folded on its rest, jaws empty.')).toBeTruthy()
    expect(within(p).getByText('5. Then Park the arm on the head-arm card.')).toBeTruthy()
    expect(within(p).getByText('Reference: seed from the bridge config')).toBeTruthy()
  })

  it('Torque off arm asks the release confirm and posts release', async () => {
    groups = [arm(false, zero())]
    mockFetch()
    render(<ServoNeutral />)
    fireEvent.click(button(await panel(), '1. Torque off arm'))
    fireEvent.click(within(dialog()).getByRole('button', { name: 'Torque off' }))
    await waitFor(() => expect(posts).toEqual([POST('release')]))
  })

  it('released: Dry run posts zero_restore_dry_run and shows the bridge message', async () => {
    groups = [arm(true, zero())]
    mockFetch()
    render(<ServoNeutral />)
    const p = await panel()
    expect(button(p, '3. Apply').disabled).toBe(true)
    fireEvent.click(button(p, '2. Dry run'))
    await waitFor(() => expect(posts).toEqual([POST('zero_restore_dry_run')]))
    expect(await screen.findByText('bridge says ok')).toBeTruthy()
  })

  it('a plan renders the table with the large-change chip', async () => {
    groups = [arm(true, zero({ plan: plan(true) }))]
    mockFetch()
    render(<ServoNeutral />)
    const p = await panel()
    const table = within(p).getByRole('table', { name: 'Zero restore plan' })
    const row = within(table).getByRole('row', { name: /shoulder_lift/ })
    expect(within(row).getByText('-0.913')).toBeTruthy()
    expect(within(row).getByText('-1.542')).toBeTruthy()
    expect(within(row).getByText('+79.2')).toBeTruthy()
    expect(within(row).getByText('large')).toBeTruthy()
    const small = within(table).getByRole('row', { name: /wrist_flex/ })
    expect(within(small).getByText('ok')).toBeTruthy()
    expect(button(p, '3. Apply').disabled).toBe(false)
  })

  it('small plan: Apply confirms with the plain dialog and posts zero_restore_apply', async () => {
    groups = [arm(true, zero({ plan: plan(false) }))]
    mockFetch()
    render(<ServoNeutral />)
    fireEvent.click(button(await panel(), '3. Apply'))
    expect(within(dialog()).queryByRole('checkbox')).toBeNull()
    fireEvent.click(within(dialog()).getByRole('button', { name: 'Apply' }))
    await waitFor(() => expect(posts).toEqual([APPLY_POST('zero_restore_apply')]))
  })

  it('large plan: Apply needs the checkbox and posts zero_restore_apply_confirmed; Cancel posts nothing', async () => {
    groups = [arm(true, zero({ plan: plan(true) }))]
    mockFetch()
    render(<ServoNeutral />)
    fireEvent.click(button(await panel(), '3. Apply'))
    expect(within(dialog()).getByText(/shoulder_lift/)).toBeTruthy()
    const confirm = within(dialog()).getByRole('button', { name: 'Apply large change' }) as HTMLButtonElement
    expect(confirm.disabled).toBe(true)
    fireEvent.click(within(dialog()).getByRole('button', { name: 'Cancel' }))
    await waitFor(() => expect(screen.queryByRole('dialog')).toBeNull())
    expect(posts).toEqual([])
    fireEvent.click(button(await panel(), '3. Apply'))
    fireEvent.click(within(dialog()).getByLabelText(DIALOG_LARGE))
    fireEvent.click(within(dialog()).getByRole('button', { name: 'Apply large change' }))
    await waitFor(() => expect(posts).toEqual([APPLY_POST('zero_restore_apply_confirmed')]))
  })

  it('a new dry run while the large confirm is open closes it and the checkbox starts unticked again', async () => {
    groups = [arm(true, zero({ plan: plan(true) }))]
    mockFetch()
    render(<ServoNeutral />)
    fireEvent.click(button(await panel(), '3. Apply'))
    fireEvent.click(within(dialog()).getByLabelText(DIALOG_LARGE))
    groups = [arm(true, zero({ plan: plan(true, 'r2') }))]
    await waitFor(() => expect(screen.queryByRole('dialog')).toBeNull(), { timeout: 3000 })
    fireEvent.click(button(await panel(), '3. Apply'))
    expect((within(dialog()).getByLabelText(DIALOG_LARGE) as HTMLInputElement).checked).toBe(false)
    fireEvent.click(within(dialog()).getByLabelText(DIALOG_LARGE))
    fireEvent.click(within(dialog()).getByRole('button', { name: 'Apply large change' }))
    await waitFor(() => expect(posts).toEqual([APPLY_POST('zero_restore_apply_confirmed', 'r2')]))
  })

  it('Apply is disabled with the reason for an expired dry run or a stale state', async () => {
    const old = new Date(Date.now() - 120_000).toISOString()
    groups = [arm(true, zero({ plan: plan(false, 'r1', old) }))]
    mockFetch()
    render(<ServoNeutral />)
    const p = await panel()
    expect(button(p, '3. Apply').disabled).toBe(true)
    expect(within(p).getByText('Dry run expired: run it again')).toBeTruthy()
    cleanup()
    groups = [arm(true, zero({ plan: plan(false) }), { stale: true })]
    render(<ServoNeutral />)
    expect(button(await panel(), '3. Apply').disabled).toBe(true)
  })

  it('shows the per-joint result and Re-enable torque posts enable', async () => {
    const lastResult = {
      run_id: 'r2',
      action: 'apply',
      finished_at: '2026-11-09T12:01:00+00:00',
      success: true,
      message: 'every joint applied and verified; torque stays off',
      record_path: '/var/lib/ros2/arm/zero_restore-x.json',
      joints: {
        shoulder_lift: { applied: true, present_after_steps: 1922, present_after_rad: -1.541, offset_after_steps: 861 },
        wrist_flex: { applied: false, present_after_steps: null, present_after_rad: null, offset_after_steps: null },
      },
      not_attempted: [],
    }
    groups = [arm(true, zero({ last_result: lastResult }))]
    mockFetch()
    render(<ServoNeutral />)
    const p = await panel()
    const table = within(p).getByRole('table', { name: 'Zero restore result' })
    const row = within(table).getByRole('row', { name: /shoulder_lift/ })
    expect(within(row).getByText('yes')).toBeTruthy()
    expect(within(row).getByText('-1.541')).toBeTruthy()
    expect(within(row).getByText('861')).toBeTruthy()
    expect(within(within(table).getByRole('row', { name: /wrist_flex/ })).getByText('no')).toBeTruthy()
    expect(within(p).getByText(/every joint applied and verified/)).toBeTruthy()
    fireEvent.click(button(p, '4. Re-enable torque'))
    await waitFor(() => expect(posts).toEqual([POST('enable')]))
  })

  it('Save current pose as rest reference confirms then posts save_rest_reference', async () => {
    groups = [arm(true, zero())]
    mockFetch()
    render(<ServoNeutral />)
    fireEvent.click(button(await panel(), 'Save current pose as rest reference'))
    fireEvent.click(within(dialog()).getByRole('button', { name: 'Save reference' }))
    await waitFor(() => expect(posts).toEqual([POST('save_rest_reference')]))
  })

  it('Save rest reference is disabled with the reason while joint_offsets is stale or torque is on', async () => {
    groups = [arm(true, zero(), { calibration_stale: ['joint_offsets', 'home'] })]
    mockFetch()
    render(<ServoNeutral />)
    const p = await panel()
    expect(button(p, 'Save current pose as rest reference').disabled).toBe(true)
    expect(within(p).getByText(/The arm zero is stale \(joint_offsets\)/)).toBeTruthy()
    cleanup()
    groups = [arm(false, zero())]
    render(<ServoNeutral />)
    const q = await panel()
    expect(button(q, 'Save current pose as rest reference').disabled).toBe(true)
    expect(within(q).getByText('Torque off the arm first.')).toBeTruthy()
  })

  it('Save rest reference is disabled while the bridge reports joint_offsets stale', async () => {
    groups = [arm(true, zero({ stale_joint_offsets: true }))]
    mockFetch()
    render(<ServoNeutral />)
    const p = await panel()
    expect(button(p, 'Save current pose as rest reference').disabled).toBe(true)
    expect(within(p).getByText(/The arm zero is stale \(joint_offsets\)/)).toBeTruthy()
  })

  it('shows a saved reference and a reference error', async () => {
    groups = [
      arm(true, zero({
        reference: { source: 'file', saved_at: '2026-11-09T10:00:00+00:00', joints: {} },
        reference_error: 'rest reference unreadable',
      })),
    ]
    mockFetch()
    render(<ServoNeutral />)
    const p = await panel()
    expect(within(p).getByText('Reference: saved 2026-11-09T10:00:00+00:00')).toBeTruthy()
    expect(within(p).getByText('rest reference unreadable')).toBeTruthy()
  })
})

describe('Move to neutral pose', () => {
  it('confirms with the exact text and posts move_to_neutral; Cancel posts nothing', async () => {
    groups = [arm(false, zero())]
    mockFetch()
    render(<ServoNeutral />)
    const card = await screen.findByRole('region', { name: 'Arm' })
    fireEvent.click(button(card, 'Move to neutral pose'))
    expect(within(dialog()).getByText(MOVE_TEXT)).toBeTruthy()
    fireEvent.click(within(dialog()).getByRole('button', { name: 'Cancel' }))
    await waitFor(() => expect(screen.queryByRole('dialog')).toBeNull())
    expect(posts).toEqual([])
    fireEvent.click(button(card, 'Move to neutral pose'))
    fireEvent.click(within(dialog()).getByRole('button', { name: 'Move' }))
    await waitFor(() => expect(posts).toEqual([POST('move_to_neutral')]))
  })

  it('is disabled while released or while joint_offsets is stale, with the reason', async () => {
    groups = [arm(true, zero())]
    mockFetch()
    render(<ServoNeutral />)
    let card = await screen.findByRole('region', { name: 'Arm' })
    expect(button(card, 'Move to neutral pose').disabled).toBe(true)
    expect(within(card).getByText('Move to neutral needs torque on.')).toBeTruthy()
    cleanup()
    groups = [arm(false, zero(), { calibration_stale: ['joint_offsets'] })]
    render(<ServoNeutral />)
    card = await screen.findByRole('region', { name: 'Arm' })
    expect(button(card, 'Move to neutral pose').disabled).toBe(true)
    expect(within(card).getByText('Move to neutral is off while the arm zero is stale (joint_offsets): restore the zero first.')).toBeTruthy()
  })

  it('is disabled while the bridge reports joint_offsets stale', async () => {
    groups = [arm(false, zero({ stale_joint_offsets: true }))]
    mockFetch()
    render(<ServoNeutral />)
    const card = await screen.findByRole('region', { name: 'Arm' })
    expect(button(card, 'Move to neutral pose').disabled).toBe(true)
    expect(within(card).getByText('Move to neutral is off while the arm zero is stale (joint_offsets): restore the zero first.')).toBeTruthy()
  })
})
