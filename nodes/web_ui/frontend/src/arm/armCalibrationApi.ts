/**
 * Client of GET /api/arm/calibration (web_ui/arm_calibration.py): the arm calibration the 3D arm views draw with (the
 * accepted arm to camera calibration, or the configured client.yml values while nothing is accepted), and the polling
 * hook that follows a new Accept without a page reload.
 */
import { useEffect, useMemo, useState } from 'react'
import log from '../logging'
import { toArmCalibration, type ArmCalibration } from './armCalibration'

export const ARM_CALIBRATION_URL = '/api/arm/calibration'
/** Poll period: the backend stats applied.json per call and reads the file only after it changed. */
export const ARM_CALIBRATION_POLL_MS = 5000
/** The five calibrated arm joints (gripper excluded), as the backend's ArmCalibrationSettings requires them. */
export const ARM_CALIBRATION_JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll'] as const

/** Where the served values come from: an accepted calibration or the configured client.yml values. */
export type ArmCalibrationSource = 'accepted' | 'configured'

/** Arm base pose in base_link (m, rad). */
export interface ArmBasePose {
  x: number
  y: number
  z: number
  yaw: number
}

/** Identity of an accepted calibration. */
export interface AcceptedArmCalibrationId {
  session: string
  result: string
  /** ISO 8601 accept time. */
  accepted_at: string
}

/** The calibration as the backend serves it. */
export interface ServedArmCalibration {
  source: ArmCalibrationSource
  /** The accepted calibration's identity, null for the configured values. */
  accepted: AcceptedArmCalibrationId | null
  joint_offsets_rad: Record<string, number>
  base_in_base_link: ArmBasePose
}

/** GET /api/arm/calibration body. */
export interface ArmCalibrationBody {
  /** The calibration in use, null only when web_ui has no arm_calibration configured. */
  calibration: ServedArmCalibration | null
  /** Why the last read failed (the values in use stay), null after a good read. */
  error: string | null
}

export type ArmCalibrationResult = { ok: true; value: ArmCalibrationBody } | { ok: false; message: string }

const isFiniteNumber = (v: unknown): v is number => typeof v === 'number' && Number.isFinite(v)
const isObject = (v: unknown): v is Record<string, unknown> => typeof v === 'object' && v !== null && !Array.isArray(v)

/**
 * Check a served calibration.
 *
 * @param v - the `calibration` value
 * @returns null when valid, otherwise the first bad key
 */
function calibrationProblem(v: unknown): string | null {
  if (!isObject(v)) return 'calibration'
  if (v.source !== 'accepted' && v.source !== 'configured') return 'calibration.source'
  if (v.source === 'configured' && v.accepted !== null) return 'calibration.accepted'
  if (v.source === 'accepted') {
    const a = v.accepted
    if (!isObject(a)) return 'calibration.accepted'
    for (const key of ['session', 'result', 'accepted_at'] as const) {
      if (typeof a[key] !== 'string') return `calibration.accepted.${key}`
    }
  }
  const offsets = v.joint_offsets_rad
  if (!isObject(offsets)) return 'calibration.joint_offsets_rad'
  for (const joint of ARM_CALIBRATION_JOINTS) {
    if (!isFiniteNumber(offsets[joint])) return `calibration.joint_offsets_rad.${joint}`
  }
  const base = v.base_in_base_link
  if (!isObject(base)) return 'calibration.base_in_base_link'
  for (const key of ['x', 'y', 'z', 'yaw'] as const) {
    if (!isFiniteNumber(base[key])) return `calibration.base_in_base_link.${key}`
  }
  return null
}

/**
 * Validate a GET /api/arm/calibration body.
 *
 * @param body - parsed JSON
 * @returns the typed body (only the drawn fields), or a message naming the first bad key
 */
export function parseArmCalibration(body: unknown): ArmCalibrationResult {
  if (!isObject(body)) return { ok: false, message: 'arm calibration: body is not an object' }
  if (body.error !== null && typeof body.error !== 'string') return { ok: false, message: 'arm calibration: bad error' }
  if (body.calibration === null) return { ok: true, value: { calibration: null, error: body.error } }
  const problem = calibrationProblem(body.calibration)
  if (problem !== null) return { ok: false, message: `arm calibration: bad ${problem}` }
  const c = body.calibration as unknown as ServedArmCalibration
  const joint_offsets_rad = Object.fromEntries(ARM_CALIBRATION_JOINTS.map((j) => [j, c.joint_offsets_rad[j]]))
  const { x, y, z, yaw } = c.base_in_base_link
  const accepted = c.accepted === null ? null : { session: c.accepted.session, result: c.accepted.result, accepted_at: c.accepted.accepted_at }
  return {
    ok: true,
    value: { calibration: { source: c.source, accepted, joint_offsets_rad, base_in_base_link: { x, y, z, yaw } }, error: body.error },
  }
}

/**
 * GET /api/arm/calibration. Never throws.
 *
 * @param fetchImpl - fetch implementation (tests)
 * @returns the validated body, or a message for network, HTTP and shape errors
 */
export async function fetchArmCalibration(fetchImpl: typeof fetch = fetch): Promise<ArmCalibrationResult> {
  let res: Response
  try {
    res = await fetchImpl(ARM_CALIBRATION_URL, { cache: 'no-store' })
  } catch (e) {
    return { ok: false, message: e instanceof Error ? e.message : String(e) }
  }
  if (!res.ok) return { ok: false, message: `arm calibration: HTTP ${res.status}` }
  let body: unknown
  try {
    body = await res.json()
  } catch {
    return { ok: false, message: 'arm calibration: body is not JSON' }
  }
  return parseArmCalibration(body)
}

/**
 * The calibration a 3D arm view draws with, polled every pollMs (while the page is visible): the accepted one or the
 * configured values, as the backend serves them. Null until the first good answer (and while web_ui has no
 * arm_calibration configured): the views draw no arm then. A failed poll keeps the last answer.
 *
 * @param pollMs - poll period in ms
 * @returns the calibration (a new object only when it changed), or null
 */
export function useArmCalibration(pollMs: number = ARM_CALIBRATION_POLL_MS): ArmCalibration | null {
  const [served, setServed] = useState<ServedArmCalibration | null>(null)
  useEffect(() => {
    let live = true
    let lastProblem: string | null = null
    const poll = async () => {
      if (document.visibilityState === 'hidden') return
      const r = await fetchArmCalibration()
      if (!live) return
      const problem = r.ok ? r.value.error : r.message
      if (problem !== null && problem !== lastProblem) log.warn('[arm] calibration:', problem)
      lastProblem = problem
      if (!r.ok) return
      const next = r.value.calibration
      setServed((prev) => (JSON.stringify(prev) === JSON.stringify(next) ? prev : next))
    }
    void poll()
    const id = setInterval(() => void poll(), pollMs)
    return () => {
      live = false
      clearInterval(id)
    }
  }, [pollMs])
  return useMemo(() => (served === null ? null : toArmCalibration(served)), [served])
}
