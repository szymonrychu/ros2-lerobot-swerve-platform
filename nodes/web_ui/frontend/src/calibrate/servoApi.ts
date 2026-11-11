/** Fetch calls and the polling hook for the servo torque / neutral section (routes /api/calibration/servos*). */
import { useEffect, useState } from 'react'

const BASE = '/api/calibration/servos'
export const SERVO_POLL_MS = 1000

/**
 * `save_confirmed` saves a recorded shift above neutral_max_shift_deg; the bridge refuses it for plain `save`.
 * `zero_restore_apply_confirmed` applies an offset change above the zero restore max_change_deg the same way.
 * `move_to_neutral` and `set_home` are served by mcp_server (/arm/move_to_neutral, /arm/set_home: the Arm tab Set
 * home), the others by the feetech bridge.
 * `calibration_store_confirmed` stores a plan that needs the confirm (needs_confirm); plain `calibration_store`
 * refuses it. Both take the shown plan's run_id as plan_run_id.
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
  | 'set_home'
  | 'calibration_record_home'
  | 'calibration_capture_start'
  | 'calibration_capture_stop'
  | 'calibration_plan'
  | 'calibration_store'
  | 'calibration_store_confirmed'

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

/** Servo calibration settings of a group (bridge `servo_calibration.config`). */
export interface ServoCalibrationConfig {
  min_range_deg: number
  margin_deg: number
  wrap_guard_deg: number
  full_turn_min_deg: number
  max_change_deg: number
  still_window_s: number
  still_tolerance_steps: number
  verify_tolerance_steps: number
  plan_max_age_s: number
  capture_status_interval_s: number
  /** Joint name -> wanted reading at home (joint radians). */
  home_reference_rad: Record<string, number>
}

/** Recorded home of the servo calibration (old frame): per joint steps and joint radians. */
export interface CalibrationHome {
  recorded_at: string
  joints: Record<string, { steps: number; rad: number }>
}

/** One joint of the running range capture. Values are null before the first sample. */
export interface CaptureJoint {
  present_steps: number | null
  /** Home plus unwrapped extremes: may fall outside 0..4095. */
  min_steps: number | null
  max_steps: number | null
  range_deg: number | null
}

/** Range capture status; the bridge sends an empty object (all fields absent) without a capture. */
export interface CalibrationCapture {
  running: boolean
  paused: boolean
  samples: number
  joints: Record<string, CaptureJoint>
}

/** One joint of the calibration plan (steps and joint radians as /follower/joint_states). */
export interface CalibrationPlanJoint {
  name: string
  id: number
  inverted: boolean
  home_steps: number
  target_steps: number
  home_rad: number
  target_rad: number
  offset_before_steps: number
  offset_change_steps: number
  change_deg: number
  new_offset_steps: number
  limits_before: [number, number]
  /** null when the limits cannot be planned (see problems). */
  limits_after: [number, number] | null
  /** Captured range in degrees; null without a capture (home-only plan). */
  range_deg: number | null
  full_turn: boolean
  /** |change| above max_change_deg: the store needs the explicit confirm. */
  exceeds_max_change: boolean
  /** limits_after narrower than the required travel: the store needs the explicit confirm. */
  narrower_than_required: boolean
  /** Reasons the store is refused; empty when the plan is storable. */
  problems: string[]
}

/** The bridge's last calibration plan, stored only while fresh (plan_max_age_s) and unmoved. */
export interface CalibrationPlan {
  run_id: string
  created_at: string
  needs_confirm: boolean
  ok: boolean
  joints: Record<string, CalibrationPlanJoint>
}

/** Per-joint outcome of the last calibration store. */
export interface CalibrationStoreJoint {
  written: boolean
  applied: boolean
  present_before_steps: number | null
  present_after_steps: number | null
  offset_after_steps: number | null
  limits_after: [number, number] | null
  rolled_back: boolean | null
  reason: string | null
}

/** Outcome of the last calibration store. */
export interface CalibrationStoreResult {
  run_id: string
  plan_run_id: string
  action: string
  finished_at: string
  success: boolean
  message: string
  record_path: string | null
  joints: Record<string, CalibrationStoreJoint>
  /** [joint name, reason] of the first failed joint; null on success. */
  failure: [string, string] | null
  not_attempted: string[]
  override_notes: string[]
  bookkeeping_errors: string[]
}

/** torque_state `servo_calibration` of a group with a servo_calibration block (bridge contract). */
export interface ServoCalibrationState {
  config: ServoCalibrationConfig
  home: CalibrationHome | null
  capture: CalibrationCapture | null
  plan: CalibrationPlan | null
  last_result: CalibrationStoreResult | null
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
  /** Servo calibration state; null or absent for a group without a servo_calibration block. */
  servo_calibration?: ServoCalibrationState | null
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
  /** The Set home step is offered (mcp_server /arm/set_home, as the Arm tab Set home). */
  set_home: boolean
  /** The servo calibration flow (record home, capture, plan, store) is offered. */
  servo_calibration: boolean
  /** Reference home pose text (web_ui home_pose_hint); null or absent: no hint. */
  home_pose_hint?: string | null
}

export interface ServoActionResult {
  ok: boolean
  message: string
  /** calibration_store / calibration_store_confirmed: the arm home was set after the store; null when not applicable. */
  home_set?: boolean | null
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
 * planRunId (zero_restore_apply*, calibration_store*): run_id of the plan the user was shown, sent as
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
