/**
 * Typed fetch calls for the arm-to-camera fit (node routes /api/arm-camera/* behind /api/calibration/rgbd/arm-camera/*).
 * Every body is shape-checked at runtime; a mismatch names the first bad key. The shapes follow the node's contract
 * fixtures (nodes/head_rgbd_calibration/tests/fixtures/api/arm_camera_*.json); armCameraContract.test.ts pins them.
 */
import { GET, POST, call, isArrayOf, isNullOr, isNum, isObj, isStr, seg, type FloorApiResult } from './headFloorApi'

const BASE = '/api/calibration/rgbd/arm-camera'
/** Suffixes the node puts on the two labels of one accuracy example. */
export const SIGMA_SUFFIX = ', 1 sigma'
export const P95_SUFFIX = ', 95 %'

/** Score of a fit; the held-out numbers are null without held-out views (non-finite numbers come as null). */
export interface ArmCameraScore {
  rms_px: number | null
  heldout_rms_px: number | null
  heldout_tool_mm: number | null
  views: number
  poses: number
}

/** An accuracy example in plain units, e.g. "tool point at 30 cm reach, 1 sigma" 0.13 mm; null when unknown. */
export interface AccuracyExample {
  label: string
  value: number | null
  unit: string
}

/** The applied head chain a fit held fixed: source floor_fit, head_arm, arm_offsets, ground_board or urdf. */
export interface ArmCameraHeadChain {
  source: string | null
  session: string | null
  result: string | null
}

/** The applied arm-camera record of the state. */
export interface ArmCameraApplied {
  session: string
  result: string
  accepted_at: string
  score: ArmCameraScore
  accuracy_examples: AccuracyExample[]
  /** The applied head chain is no longer the one this calibration was fitted against. */
  head_chain_changed: boolean
}

/** Body of GET /arm-camera/state: what the fit would start from and whether it may run (`reason` when not). */
export interface ArmCameraState {
  /** The applied arm-camera record, null while none is applied. */
  applied: ArmCameraApplied | null
  /** Which fit wrote the applied head chain (floor_fit, head_arm, arm_offsets, ground_board, urdf), null when unknown. */
  head_chain_source: string | null
  ready: boolean
  reason: string | null
}

/** Answer of POST .../compute (202): the id of the result being computed. */
export interface ArmCameraCompute {
  result_id: string
}

/**
 * One fitted parameter in deg or mm (the sag in deg/(N m)) with its 1-sigma, the applied value and the change from it.
 * `held`: the gauge holds it (not fitted, sigma null); sigma is also null for a fitted parameter whose sigma is not
 * finite (a singular covariance), then held is false.
 */
export interface ArmCameraParam {
  name: string
  value: number | null
  sigma: number | null
  held: boolean
  unit: string
  applied_value: number | null
  change: number | null
}

/** A refusal or a warning: the checked value against its limit, with the node's message. */
export interface ArmCameraCheck {
  name: string
  value: number | null
  limit: number | null
  unit: string
  message: string
}

/** Comparison with the calibration in effect: its score on this session and how far the tool would move. */
export interface ArmCameraComparison {
  applied: Record<string, unknown> | null
  current_score: { rms_px: number | null; heldout_rms_px: number | null; heldout_tool_mm: number | null }
  shift_examples: AccuracyExample[]
}

/** The quality report of one arm-camera fit. */
export interface ArmCameraReport {
  plausible: boolean
  refusals: ArmCameraCheck[]
  warnings: ArmCameraCheck[]
  score: ArmCameraScore
  params: ArmCameraParam[]
  /** Two entries per example: "<example>, 1 sigma" and "<example>, 95 %". */
  accuracy_examples: AccuracyExample[]
  comparison: ArmCameraComparison
  skew: { xyz_mm: (number | null)[]; rpy_deg: (number | null)[] }
  sag: Record<string, unknown>
  /** Gauge and model notes of the fit. */
  notes: string[]
}

export type ArmCameraResultState = 'running' | 'done' | 'failed'

/** GET .../results/{rid}: the report and calibration are present once the state is done. */
export interface ArmCameraResult {
  id: string
  session: string
  kind: 'arm_camera'
  state: ArmCameraResultState
  error: string | null
  created_at: string
  declined: boolean
  declined_at: string | null
  accepted_at: string | null
  head_chain: ArmCameraHeadChain | null
  report: ArmCameraReport | null
  /** The fitted calibration (joint_offsets_rad, base_in_base_link, tool_offset_m, snippet), null until done. */
  calibration: Record<string, unknown> | null
}

/** A consumer unit restarted by an accept. */
export interface ArmCameraUnit {
  name: string
  active_state: string
  restarted: boolean
}

/** Answer of accept: the record written to applied/arm_calibration.json and the restarted units. */
export interface ArmCameraAccept {
  arm_calibration: { session: string; result: string; accepted_at: string; score: ArmCameraScore }
  units: ArmCameraUnit[]
}

/** A runtime check of one value. */
export type Check = (v: unknown) => boolean
/** A body shape: one check per key. */
export type Shape = Record<string, Check>
export const isBool: Check = (v) => typeof v === 'boolean'
/** A finite number or null (the node sends non-finite numbers as null). */
export const isNum0: Check = isNullOr(isNum)
const RESULT_STATES: ArmCameraResultState[] = ['running', 'done', 'failed']

/** Name of the first key of `v` that is missing or has the wrong type (`a.b` for nested shapes), or null. */
export function badKey(v: unknown, shape: Shape): string | null {
  if (!isObj(v)) return '(body)'
  return Object.keys(shape).find((k) => !(k in v) || !shape[k](v[k])) ?? null
}

/** A check that the value fits `shape`. */
export const fits =
  (shape: Shape): Check =>
  (v) =>
    badKey(v, shape) === null

const SCORE: Shape = { rms_px: isNum0, heldout_rms_px: isNum0, heldout_tool_mm: isNum0, views: isNum, poses: isNum }
const EXAMPLE: Shape = { label: isStr, value: isNum0, unit: isStr }
const HEAD_CHAIN: Shape = { source: isNullOr(isStr), session: isNullOr(isStr), result: isNullOr(isStr) }
const APPLIED: Shape = {
  session: isStr,
  result: isStr,
  accepted_at: isStr,
  score: fits(SCORE),
  accuracy_examples: isArrayOf(fits(EXAMPLE)),
  head_chain_changed: isBool,
}
const STATE: Shape = {
  applied: isNullOr(fits(APPLIED)),
  head_chain_source: isNullOr(isStr),
  ready: isBool,
  reason: isNullOr(isStr),
}
const PARAM: Shape = {
  name: isStr,
  value: isNum0,
  sigma: isNum0,
  held: isBool,
  unit: isStr,
  applied_value: isNum0,
  change: isNum0,
}
const CHECK: Shape = { name: isStr, value: isNum0, limit: isNum0, unit: isStr, message: isStr }
const CURRENT_SCORE: Shape = { rms_px: isNum0, heldout_rms_px: isNum0, heldout_tool_mm: isNum0 }
const COMPARISON: Shape = {
  applied: isNullOr(isObj),
  current_score: fits(CURRENT_SCORE),
  shift_examples: isArrayOf(fits(EXAMPLE)),
}
const SKEW: Shape = { xyz_mm: isArrayOf(isNum0), rpy_deg: isArrayOf(isNum0) }
const REPORT: Shape = {
  plausible: isBool,
  refusals: isArrayOf(fits(CHECK)),
  warnings: isArrayOf(fits(CHECK)),
  score: fits(SCORE),
  params: isArrayOf(fits(PARAM)),
  accuracy_examples: isArrayOf(fits(EXAMPLE)),
  comparison: fits(COMPARISON),
  skew: fits(SKEW),
  sag: isObj,
  notes: isArrayOf(isStr),
}
const RESULT: Shape = {
  id: isStr,
  session: isStr,
  kind: (k) => k === 'arm_camera',
  state: (s) => RESULT_STATES.includes(s as ArmCameraResultState),
  error: isNullOr(isStr),
  created_at: isStr,
  declined: isBool,
  declined_at: isNullOr(isStr),
  accepted_at: isNullOr(isStr),
  head_chain: isNullOr(fits(HEAD_CHAIN)),
  report: () => true, // checked by badResult with the key path
  calibration: isNullOr(isObj),
}
const UNIT: Shape = { name: isStr, active_state: isStr, restarted: isBool }
const ACCEPT_RECORD: Shape = { session: isStr, result: isStr, accepted_at: isStr, score: fits(SCORE) }
const ACCEPT: Shape = { arm_calibration: fits(ACCEPT_RECORD), units: isArrayOf(fits(UNIT)) }

/** Every shape the client checks, by name; the contract test compares their keys with the node's fixtures. */
export const ARM_CAMERA_SHAPES: Record<string, Shape> = {
  score: SCORE,
  example: EXAMPLE,
  headChain: HEAD_CHAIN,
  applied: APPLIED,
  state: STATE,
  param: PARAM,
  check: CHECK,
  currentScore: CURRENT_SCORE,
  comparison: COMPARISON,
  skew: SKEW,
  report: REPORT,
  result: RESULT,
  unit: UNIT,
  acceptRecord: ACCEPT_RECORD,
  accept: ACCEPT,
}

/** First problem of a state body as a key name, or null. */
export const badState = (v: unknown): string | null => {
  const base = badKey(v, STATE)
  if (base !== 'applied' || !isObj(v)) return base
  return `applied.${badKey(v.applied, APPLIED)}`
}

/** First problem of a result body as a key name (`report.score`), or null. A done result needs its report. */
export function badResult(v: unknown): string | null {
  const base = badKey(v, RESULT)
  if (base !== null || !isObj(v)) return base
  if (v.report === null) return v.state === 'done' ? 'report' : null
  const report = badKey(v.report, REPORT)
  return report === null ? null : `report.${report}`
}

/** First problem of an accept answer as a key name, or null. */
export const badAccept = (v: unknown): string | null => badKey(v, ACCEPT)

async function request<T>(url: string, init: RequestInit, bad: (v: unknown) => string | null): Promise<FloorApiResult<T>> {
  const r = await call<unknown>(url, init, () => true)
  if (!r.ok) return r
  const key = bad(r.value)
  return key === null
    ? { ok: true, value: r.value as T }
    : { ok: false, status: 200, message: `Unexpected arm-camera response from the robot: missing or invalid ${key}` }
}

/** One accuracy example with both of its values (null when the node did not send that one). */
export interface AccuracyPair {
  label: string
  sigma: number | null
  p95: number | null
  unit: string
}

/**
 * The node's accuracy examples paired by example: "<example>, 1 sigma" and "<example>, 95 %" become one entry, in
 * the order the examples first appear. A label without either suffix is taken as a 1-sigma value.
 *
 * @param examples - accuracy_examples of a report or of the applied record
 * @returns one pair per example
 */
export function pairAccuracy(examples: AccuracyExample[]): AccuracyPair[] {
  const pairs = new Map<string, AccuracyPair>()
  for (const e of examples) {
    const p95 = e.label.endsWith(P95_SUFFIX)
    const suffix = p95 ? P95_SUFFIX : e.label.endsWith(SIGMA_SUFFIX) ? SIGMA_SUFFIX : ''
    const label = e.label.slice(0, e.label.length - suffix.length)
    const pair = pairs.get(label) ?? { label, sigma: null, p95: null, unit: e.unit }
    if (p95) pair.p95 = e.value
    else pair.sigma = e.value
    pairs.set(label, pair)
  }
  return [...pairs.values()]
}

const resultPath = (sid: string, rid: string) => `${BASE}/sessions/${seg(sid)}/results/${seg(rid)}`

export const getArmCameraState = () => request<ArmCameraState>(`${BASE}/state`, GET, badState)
/** Starts the fit of a head-arm capture session; the result is polled with getArmCameraResult. */
export const computeArmCamera = (sid: string) =>
  request<ArmCameraCompute>(`${BASE}/sessions/${seg(sid)}/compute`, POST, (v) => badKey(v, { result_id: isStr }))
export const getArmCameraResult = (sid: string, rid: string) => request<ArmCameraResult>(resultPath(sid, rid), GET, badResult)
/** Accepts (applies) a plausible result; a refusal (409) comes back with the node's message. */
export const acceptArmCamera = (sid: string, rid: string) =>
  request<ArmCameraAccept>(`${resultPath(sid, rid)}/accept`, POST, badAccept)
/** Declines a result; the answer is the result with declined true. */
export const declineArmCamera = (sid: string, rid: string) =>
  request<ArmCameraResult>(`${resultPath(sid, rid)}/decline`, POST, badResult)
