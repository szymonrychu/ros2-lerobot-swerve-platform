/**
 * Typed fetch calls for the arm-to-camera fit (node routes /api/arm-camera/* behind /api/calibration/rgbd/arm-camera/*).
 * Every body is shape-checked at runtime; a mismatch names the first bad key.
 */
import { GET, POST, call, isArrayOf, isNullOr, isNum, isObj, isStr, seg, type FloorApiResult } from './headFloorApi'

const BASE = '/api/calibration/rgbd/arm-camera'

/** Body of GET /arm-camera/state: what the fit would start from and whether it may run (`reason` when not). */
export interface ArmCameraState {
  /** The applied arm-camera record, null while none is applied. */
  applied: Record<string, unknown> | null
  /** Which fit wrote the applied head chain (floor, head_arm, arm), null when none is applied. */
  head_chain_source: string | null
  ready: boolean
  reason: string | null
}

/** Answer of POST .../compute (202): the id of the result being computed. */
export interface ArmCameraCompute {
  result_id: string
}

/** One fitted parameter with its 1-sigma, unit, the applied value and the change from it (null when none is applied). */
export interface ArmCameraParam {
  name: string
  value: number
  sigma: number
  unit: string
  applied_value: number | null
  change: number | null
}

/** An accuracy example in plain units, e.g. "tool point at 30 cm reach: +-" 1.9 mm. */
export interface AccuracyExample {
  label: string
  value: number
  unit: string
}

export interface ArmCameraScore {
  rms_px: number
  heldout_rms_px: number
  heldout_tool_mm: number
  views: number
  poses: number
}

/** The quality report of one arm-camera fit. */
export interface ArmCameraReport {
  plausible: boolean
  refusals: string[]
  warnings: string[]
  score: ArmCameraScore
  params: ArmCameraParam[]
  accuracy_examples: AccuracyExample[]
  /** Free-form comparison with the current calibration: key -> {current, new} or a value. */
  comparison: Record<string, unknown>
  skew: { xyz_mm: number[]; rpy_deg: number[] }
  sag: Record<string, unknown>
}

export type ArmCameraResultState = 'running' | 'done' | 'failed'

/** GET .../results/{rid}: the report is present once the state is done. */
export interface ArmCameraResult {
  id: string
  session: string
  state: ArmCameraResultState
  error: string | null
  created_at: string
  report: ArmCameraReport | null
}

/** Answer of accept / decline: the node's message when it sends one. */
export interface ArmCameraDecision {
  ok?: boolean
  message?: string
}

type Check = (v: unknown) => boolean
const isBool: Check = (v) => typeof v === 'boolean'
const RESULT_STATES: ArmCameraResultState[] = ['running', 'done', 'failed']

/** Name of the first key of `v` that is missing or has the wrong type, or null when all fit. */
function badKey(v: unknown, shape: Record<string, Check>): string | null {
  if (!isObj(v)) return '(body)'
  return Object.keys(shape).find((k) => !(k in v) || !shape[k](v[k])) ?? null
}

const STATE: Record<string, Check> = {
  applied: isNullOr(isObj),
  head_chain_source: isNullOr(isStr),
  ready: isBool,
  reason: isNullOr(isStr),
}
const PARAM: Record<string, Check> = {
  name: isStr,
  value: isNum,
  sigma: isNum,
  unit: isStr,
  applied_value: isNullOr(isNum),
  change: isNullOr(isNum),
}
const SCORE: Record<string, Check> = { rms_px: isNum, heldout_rms_px: isNum, heldout_tool_mm: isNum, views: isNum, poses: isNum }
const EXAMPLE: Record<string, Check> = { label: isStr, value: isNum, unit: isStr }
const REPORT: Record<string, Check> = {
  plausible: isBool,
  refusals: isArrayOf(isStr),
  warnings: isArrayOf(isStr),
  score: (s) => badKey(s, SCORE) === null,
  params: isArrayOf((p) => badKey(p, PARAM) === null),
  accuracy_examples: isArrayOf((e) => badKey(e, EXAMPLE) === null),
  comparison: isObj,
  skew: (s) => isObj(s) && isArrayOf(isNum)(s.xyz_mm) && isArrayOf(isNum)(s.rpy_deg),
  sag: isObj,
}
const RESULT: Record<string, Check> = {
  id: isStr,
  session: isStr,
  state: (s) => RESULT_STATES.includes(s as ArmCameraResultState),
  error: isNullOr(isStr),
  created_at: isStr,
}

/** First problem of a result body as a key name (`report.score`), or null. A done result needs its report. */
export function badResult(v: unknown): string | null {
  const base = badKey(v, RESULT)
  if (base !== null || !isObj(v)) return base
  if (v.report === null || v.report === undefined) return v.state === 'done' ? 'report' : null
  const report = badKey(v.report, REPORT)
  return report === null ? null : `report.${report}`
}

async function request<T>(url: string, init: RequestInit, bad: (v: unknown) => string | null): Promise<FloorApiResult<T>> {
  const r = await call<unknown>(url, init, () => true)
  if (!r.ok) return r
  const key = bad(r.value)
  return key === null
    ? { ok: true, value: r.value as T }
    : { ok: false, status: 200, message: `Unexpected arm-camera response from the robot: missing or invalid ${key}` }
}

const resultPath = (sid: string, rid: string) => `${BASE}/sessions/${seg(sid)}/results/${seg(rid)}`

export const getArmCameraState = () => request<ArmCameraState>(`${BASE}/state`, GET, (v) => badKey(v, STATE))
/** Starts the fit of a head-arm capture session; the result is polled with getArmCameraResult. */
export const computeArmCamera = (sid: string) =>
  request<ArmCameraCompute>(`${BASE}/sessions/${seg(sid)}/compute`, POST, (v) => badKey(v, { result_id: isStr }))
export const getArmCameraResult = (sid: string, rid: string) => request<ArmCameraResult>(resultPath(sid, rid), GET, badResult)
/** Accepts (applies) a plausible result; a refusal (409) comes back with the node's message. */
export const acceptArmCamera = (sid: string, rid: string) =>
  request<ArmCameraDecision>(`${resultPath(sid, rid)}/accept`, POST, () => null)
export const declineArmCamera = (sid: string, rid: string) =>
  request<ArmCameraDecision>(`${resultPath(sid, rid)}/decline`, POST, () => null)
