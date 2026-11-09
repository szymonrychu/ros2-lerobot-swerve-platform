/** Fetch calls and the polling hook for the servo torque / neutral section (routes /api/calibration/servos*). */
import { useEffect, useState } from 'react'

const BASE = '/api/calibration/servos'
export const SERVO_POLL_MS = 1000

/**
 * `save_confirmed` saves a recorded shift above neutral_max_shift_deg; the bridge refuses it for plain `save`.
 * `zero_restore_apply_confirmed` applies an offset change above the zero restore max_change_deg the same way.
 * `move_to_neutral` is served by mcp_server (/arm/move_to_neutral), the others by the feetech bridge.
 */
export type ServoAction =
  | 'release'
  | 'enable'
  | 'record'
  | 'save'
  | 'save_confirmed'
  | 'zero_restore_dry_run'
  | 'zero_restore_apply'
  | 'zero_restore_apply_confirmed'
  | 'save_rest_reference'
  | 'move_to_neutral'

/** One joint of a zero restore dry run plan (steps and joint radians as /follower/joint_states). */
export interface ZeroRestorePlanJoint {
  present_steps: number
  present_rad: number
  target_steps: number
  target_rad: number
  offset_before_steps: number
  offset_change_steps: number
  change_deg: number
  new_offset_steps: number
  /** |change| above max_change_deg: the apply needs the explicit confirm. */
  exceeds_max_change: boolean
  /** (min, max) angle limits read by the dry run (restored after the write when they moved). */
  limits: [number, number]
}

/** The bridge's last zero restore dry run, applied only while fresh (max_age_s) and unmoved. */
export interface ZeroRestorePlan {
  run_id: string
  created_at: string
  max_age_s: number
  reference_source: 'file' | 'seed'
  needs_confirm: boolean
  joints: Record<string, ZeroRestorePlanJoint>
}

/** Per-joint outcome of the last zero restore apply. */
export interface ZeroRestoreResultJoint {
  applied: boolean
  present_after_steps: number | null
  present_after_rad: number | null
  offset_after_steps: number | null
}

export interface ZeroRestoreResult {
  run_id: string
  action: string
  finished_at: string
  success: boolean
  message: string
  record_path: string | null
  joints: Record<string, ZeroRestoreResultJoint>
  not_attempted: string[]
}

/** torque_state `zero_restore` of a group with a zero_restore block (bridge contract). */
export interface ZeroRestoreState {
  max_change_deg: number
  reference: { source: 'file' | 'seed'; saved_at: string | null; joints: Record<string, number> } | null
  reference_error: string | null
  stale_joint_offsets: boolean
  plan: ZeroRestorePlan | null
  last_result: ZeroRestoreResult | null
}

/** Recorded neutral of one joint, not saved yet. */
export interface NeutralCaptureJoint {
  steps: number
  delta_steps: number
  /** The shift exceeds the group's neutral_max_shift_deg: saving it needs the explicit confirm (save_confirmed). */
  exceeds_max_shift: boolean
}

/** The bridge's /<ns>/torque_state sample. */
export interface TorqueState {
  namespace: string
  released: boolean
  /** Joint name -> torque_enable reading (0 or 1). */
  joints: Record<string, number>
  stamp_ns: number
  neutral_enabled: boolean
  neutral_capture: Record<string, NeutralCaptureJoint> | null
  last_save: { saved_at?: unknown; saved?: unknown; failed?: unknown } | null
  /** Largest recorded shift (deg) saved without the large-shift confirm; null for a group without neutral. */
  neutral_max_shift_deg: number | null
  /** Zero restore state; null or absent for a group without a zero_restore block. */
  zero_restore?: ZeroRestoreState | null
}

/** One entry of GET /api/calibration/servos. */
export interface ServoGroup {
  id: string
  label: string
  neutral: boolean
  state: TorqueState | null
  age_s: number | null
  stale: boolean
  calibration_stale: string[] | null
  /** The zero restore actions are offered (Arm zero restore panel). */
  zero_restore: boolean
  /** The Move to neutral pose button is offered (mcp_server /arm/move_to_neutral). */
  move_to_neutral: boolean
}

export interface ServoActionResult {
  ok: boolean
  message: string
}

export type ServoApiResult<T> = { ok: true; value: T } | { ok: false; status: number; message: string }

function isGroup(v: unknown): v is ServoGroup {
  const g = v as Partial<ServoGroup> | null
  return !!g && typeof g.id === 'string' && typeof g.label === 'string' && typeof g.neutral === 'boolean'
}

async function call<T>(url: string, init: RequestInit, parse: (body: unknown) => T | null): Promise<ServoApiResult<T>> {
  let res: Response
  try {
    res = await fetch(url, init)
  } catch (e) {
    return { ok: false, status: 0, message: e instanceof Error ? e.message : String(e) }
  }
  let body: unknown = null
  try {
    body = await res.json()
  } catch {
    body = null
  }
  if (res.ok) {
    const value = parse(body)
    return value === null
      ? { ok: false, status: res.status, message: 'Unexpected servo response from the robot' }
      : { ok: true, value }
  }
  const b = (body ?? {}) as { message?: unknown; reason?: unknown }
  const message = typeof b.message === 'string' ? b.message : typeof b.reason === 'string' ? b.reason : `HTTP ${res.status}`
  return { ok: false, status: res.status, message }
}

/** GET /api/calibration/servos. */
export const fetchServoGroups = () =>
  call<ServoGroup[]>(BASE, { cache: 'no-store' }, (b) => {
    const groups = (b as { groups?: unknown } | null)?.groups
    return Array.isArray(groups) && groups.every(isGroup) ? groups : null
  })

/**
 * POST /api/calibration/servos/{groupId}/{action}; a refusal (409) comes back as ok:false with the backend message.
 * planRunId (zero_restore_apply / zero_restore_apply_confirmed): run_id of the plan the user was shown, sent as
 * plan_run_id; the backend refuses the apply when another dry run replaced it.
 */
export const servoAction = (groupId: string, action: ServoAction, planRunId?: string) =>
  call<ServoActionResult>(
    `${BASE}/${groupId}/${action}${planRunId === undefined ? '' : `?plan_run_id=${encodeURIComponent(planRunId)}`}`,
    { method: 'POST' },
    (b) => (typeof (b as ServoActionResult | null)?.message === 'string' ? (b as ServoActionResult) : null),
  )

export interface ServoGroupsState {
  groups: ServoGroup[]
  /** True once the backend answered 404: no servo groups configured here. */
  notConfigured: boolean
  /** Latest network or server error, null when the last poll succeeded. */
  error: string | null
  loading: boolean
}

/** Polls the servo groups every pollMs while the tab is visible. 404 means not configured, not an error. */
export function useServoGroups(pollMs: number = SERVO_POLL_MS): ServoGroupsState {
  const [state, setState] = useState<ServoGroupsState>({ groups: [], notConfigured: false, error: null, loading: true })
  useEffect(() => {
    let live = true
    const poll = async () => {
      if (document.visibilityState === 'hidden') return
      const r = await fetchServoGroups()
      if (!live) return
      if (r.ok) setState({ groups: r.value, notConfigured: false, error: null, loading: false })
      else if (r.status === 404) setState({ groups: [], notConfigured: true, error: null, loading: false })
      else setState((s) => ({ ...s, error: r.message, loading: false }))
    }
    void poll()
    const id = setInterval(() => void poll(), pollMs)
    return () => {
      live = false
      clearInterval(id)
    }
  }, [pollMs])
  return state
}
