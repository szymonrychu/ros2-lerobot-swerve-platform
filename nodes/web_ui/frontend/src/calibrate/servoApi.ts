/** Fetch calls and the polling hook for the servo torque / neutral section (routes /api/calibration/servos*). */
import { useEffect, useState } from 'react'

const BASE = '/api/calibration/servos'
export const SERVO_POLL_MS = 1000

export type ServoAction = 'release' | 'enable' | 'record' | 'save'

/** Recorded neutral of one joint, not saved yet. */
export interface NeutralCaptureJoint {
  steps: number
  delta_steps: number
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

/** POST /api/calibration/servos/{groupId}/{action}; a refusal (409) comes back as ok:false with the backend message. */
export const servoAction = (groupId: string, action: ServoAction) =>
  call<ServoActionResult>(`${BASE}/${groupId}/${action}`, { method: 'POST' }, (b) =>
    typeof (b as ServoActionResult | null)?.message === 'string' ? (b as ServoActionResult) : null,
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
