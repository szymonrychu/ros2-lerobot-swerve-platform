/**
 * Client of GET /api/arm/calibration (web_ui/arm_calibration.py): the accepted arm to camera calibration the 3D arm
 * views draw with, and the polling hook that follows a new Accept without a page reload.
 */
import { useEffect, useMemo, useState } from 'react'
import log from '../logging'
import { effectiveArmCalibration, type ArmCalibration } from './armCalibration'

export const ARM_CALIBRATION_URL = '/api/arm/calibration'
/** Poll period: the backend stats applied.json per call and reads the file only after it changed. */
export const ARM_CALIBRATION_POLL_MS = 5000
/** The five calibrated arm joints (gripper excluded), as the backend's ArmCalibrationFile requires them. */
export const ARM_CALIBRATION_JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll'] as const

/** Arm base pose in base_link (m, rad). */
export interface ArmBasePose {
  x: number
  y: number
  z: number
  yaw: number
}

/** The accepted calibration as the backend serves it. */
export interface AcceptedArmCalibration {
  session: string
  result: string
  /** ISO 8601 accept time. */
  accepted_at: string
  joint_offsets_rad: Record<string, number>
  base_in_base_link: ArmBasePose
}

/** GET /api/arm/calibration body. */
export interface ArmCalibrationBody {
  /** The accepted calibration in use, null when nothing is accepted (or no applied dir is configured). */
  accepted: AcceptedArmCalibration | null
  /** Why the last read failed (the last good calibration stays), null after a good read. */
  error: string | null
}

export type ArmCalibrationResult = { ok: true; value: ArmCalibrationBody } | { ok: false; message: string }

const isFiniteNumber = (v: unknown): v is number => typeof v === 'number' && Number.isFinite(v)
const isObject = (v: unknown): v is Record<string, unknown> => typeof v === 'object' && v !== null && !Array.isArray(v)

/**
 * Check an accepted calibration body.
 *
 * @param v - the `accepted` value
 * @returns null when valid, otherwise the first bad key
 */
function acceptedProblem(v: unknown): string | null {
  if (!isObject(v)) return 'accepted'
  for (const key of ['session', 'result', 'accepted_at'] as const) {
    if (typeof v[key] !== 'string') return `accepted.${key}`
  }
  const offsets = v.joint_offsets_rad
  if (!isObject(offsets)) return 'accepted.joint_offsets_rad'
  for (const joint of ARM_CALIBRATION_JOINTS) {
    if (!isFiniteNumber(offsets[joint])) return `accepted.joint_offsets_rad.${joint}`
  }
  const base = v.base_in_base_link
  if (!isObject(base)) return 'accepted.base_in_base_link'
  for (const key of ['x', 'y', 'z', 'yaw'] as const) {
    if (!isFiniteNumber(base[key])) return `accepted.base_in_base_link.${key}`
  }
  return null
}

/**
 * Validate a GET /api/arm/calibration body.
 *
 * @param body - parsed JSON
 * @returns the typed body, or a message naming the first bad key
 */
export function parseArmCalibration(body: unknown): { ok: true; value: ArmCalibrationBody } | { ok: false; message: string } {
  if (!isObject(body)) return { ok: false, message: 'arm calibration: body is not an object' }
  if (body.error !== null && typeof body.error !== 'string') return { ok: false, message: 'arm calibration: bad error' }
  if (body.accepted === null) return { ok: true, value: { accepted: null, error: body.error } }
  const problem = acceptedProblem(body.accepted)
  if (problem !== null) return { ok: false, message: `arm calibration: bad ${problem}` }
  const a = body.accepted as unknown as AcceptedArmCalibration
  const joint_offsets_rad = Object.fromEntries(ARM_CALIBRATION_JOINTS.map((j) => [j, a.joint_offsets_rad[j]]))
  const { x, y, z, yaw } = a.base_in_base_link
  return {
    ok: true,
    value: {
      accepted: { session: a.session, result: a.result, accepted_at: a.accepted_at, joint_offsets_rad, base_in_base_link: { x, y, z, yaw } },
      error: body.error,
    },
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
 * The calibration a 3D arm view draws with, polled every pollMs (while the page is visible). Until the first answer,
 * and while the backend has nothing accepted, it is the tab's configured mount; a failed poll keeps the last answer.
 *
 * @param armOffset - tab arm_offset [x, y, z] metres, undefined for the default mount
 * @param pollMs - poll period in ms
 * @returns the calibration (a new object only when it changed)
 */
export function useArmCalibration(armOffset?: readonly [number, number, number], pollMs: number = ARM_CALIBRATION_POLL_MS): ArmCalibration {
  const [accepted, setAccepted] = useState<AcceptedArmCalibration | null>(null)
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
      const next = r.value.accepted
      setAccepted((prev) => (JSON.stringify(prev) === JSON.stringify(next) ? prev : next))
    }
    void poll()
    const id = setInterval(() => void poll(), pollMs)
    return () => {
      live = false
      clearInterval(id)
    }
  }, [pollMs])
  const [x, y, z] = armOffset ?? [undefined, undefined, undefined]
  return useMemo(
    () => effectiveArmCalibration(x === undefined || y === undefined || z === undefined ? undefined : [x, y, z], accepted),
    [x, y, z, accepted],
  )
}
