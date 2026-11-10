/**
 * Typed fetch calls for the camera to ground fit from the big ChArUco board on the floor (node routes
 * /api/ground-board/* behind /api/calibration/rgbd/ground-board/*). Every body is shape-checked at runtime; a mismatch
 * names the first bad key. The shapes follow the node's contract fixtures
 * (nodes/head_rgbd_calibration/tests/fixtures/api/ground_board_*.json); groundBoardContract.test.ts pins them.
 */
import {
  type AccuracyExample,
  type ArmCameraCheck,
  type ArmCameraUnit,
  type Check,
  type Shape,
  badKey,
  fits,
  isBool,
  isNum0,
} from './armCameraApi'
import { GET, POST, call, isArrayOf, isNullOr, isNum, isObj, isStr, seg, type FloorApiResult } from './headFloorApi'

const BASE = '/api/calibration/rgbd/ground-board'
/**
 * The gauge of the fit (shared ground_board_fit GAUGE_PARAMS): the head mount x, y and yaw are held at the applied
 * values, reported with sigma null and change 0.
 */
export const GROUND_GAUGE_PARAMS = ['mount_x_m', 'mount_y_m', 'mount_yaw_deg']

export type GroundSweepStatus =
  | 'idle'
  | 'starting'
  | 'homing'
  | 'locating'
  | 'sweeping'
  | 'awaiting_placement'
  | 'done'
  | 'stopped'
  | 'error'

/** The located board of the current placement on the floor. */
export interface GroundBoardPlace {
  x_m: number
  y_m: number
  yaw_deg: number
}

/**
 * Ground session state. `placement` is the 0-based placement being captured (the next one while awaiting), `pose` /
 * `total` the 1-based grid pose of it (home included), `message` the operator hint (a placement failure reason or
 * "move the board ...").
 */
export interface GroundSweep {
  running: boolean
  status: GroundSweepStatus
  session_id: string | null
  placement: number
  placements_done: number
  max_placements: number
  awaiting_placement: boolean
  pose: number
  total: number
  views: number
  planes: number
  skipped: number
  board: GroundBoardPlace | null
  message: string | null
  error: string | null
}

/** A head chain or result named by session and result; source floor_fit, head_arm, arm_offsets, ground_board, urdf. */
export interface GroundHeadChain {
  source: string | null
  session: string | null
  result: string | null
}

/** One verification check: the median value against its limit. */
export interface GroundVerifyCheck {
  name: string
  value: number | null
  limit: number | null
  ok: boolean
}

/** One judged verification pose: the /ground/state medians at the pose and its checks. */
export interface GroundVerifyPose {
  pose: number
  pan_deg: number
  tilt_deg: number
  reached: { pan_deg: number; tilt_deg: number } | null
  samples: number
  plane_samples: number
  height_m: number | null
  plane_vs_base_deg: number | null
  obstacle_points: number | null
  obstacle_points_max: number | null
  drop_points: number | null
  drop_points_max: number | null
  checks: GroundVerifyCheck[]
  passed: boolean
  /** "did not reach pan/tilt ..." or null. */
  error: string | null
}

export type GroundVerifyStatus = 'idle' | 'starting' | 'waiting_for_restarts' | 'measuring' | 'done' | 'stopped' | 'error'

/** A stored verification (with an accepted result): the verification state without `running`. */
export interface GroundVerify {
  status: GroundVerifyStatus
  id: string | null
  head_chain: GroundHeadChain | null
  /** The ground board result the summary is stored with, null for another head chain. */
  result: { session: string; result: string } | null
  pose: number
  total: number
  poses: GroundVerifyPose[]
  /** Null until every pose was judged. */
  passed: boolean | null
  error: string | null
}

/** GET/POST /verify/*: the verification state. */
export interface GroundVerifyState extends GroundVerify {
  running: boolean
}

/** The ground compute job. */
export interface GroundJob {
  running: boolean
  session_id: string | null
  result_id: string | null
  started_at: string | null
  error: string | null
}

/** Score of a ground board fit (held-out null without held-out views; the depth cross-check null without planes). */
export interface GroundScore {
  rms_px: number | null
  heldout_rms_px: number | null
  views: number
  poses: number
  placements: number
  floor_height_rms_mm: number | null
  floor_normal_rms_deg: number | null
}

/** The ground board result whose head chain is applied. */
export interface GroundAccepted {
  session: string
  result: string
  accepted_at: string
  score: GroundScore
  accuracy_examples: AccuracyExample[]
  verification: GroundVerify | null
}

/** Body of GET /ground-board/state. */
export interface GroundState {
  sweep: GroundSweep
  verify: GroundVerifyState
  compute: GroundJob
  /** The head chain in effect. */
  head_chain: GroundHeadChain | null
  accepted: GroundAccepted | null
}

/** One head parameter in mm or deg; a gauge parameter has sigma null and change 0 (see GROUND_GAUGE_PARAMS). */
export interface GroundParam {
  name: string
  value: number | null
  sigma: number | null
  unit: string
  applied_value: number | null
  change: number | null
}

/** One board placement as fitted. */
export interface GroundPlacement {
  placement: number
  x_m: number | null
  y_m: number | null
  yaw_deg: number | null
  sigma_x_mm: number | null
  sigma_y_mm: number | null
  sigma_yaw_deg: number | null
  views: number
  rms_px: number | null
}

/** The depth floor planes under a head chain: pose-mean height and normal RMS. */
export interface GroundPlaneScore {
  height_rms_mm: number | null
  normal_rms_deg: number | null
}

/** One head pose of the fit. */
export interface GroundPose {
  pose: number
  placement: number
  pan_deg: number
  tilt_deg: number
  views: number
  rms_px: number | null
  held_out: boolean
}

/** The quality report of one ground board fit. */
export interface GroundReport {
  plausible: boolean
  refusals: ArmCameraCheck[]
  warnings: ArmCameraCheck[]
  score: GroundScore
  params: GroundParam[]
  placements: GroundPlacement[]
  /** Two entries per example: "<example>, 1 sigma" and "<example>, 95 %". */
  accuracy_examples: AccuracyExample[]
  comparison: {
    applied: GroundHeadChain | null
    current_score: { rms_px: number | null; heldout_rms_px: number | null }
    shift_examples: AccuracyExample[]
  }
  cross_check: { planes: number; poses: number; fitted: GroundPlaneScore; applied: GroundPlaneScore }
  poses: GroundPose[]
  notes: string[]
}

export type GroundResultState = 'running' | 'done' | 'failed'

/** GET .../results/{rid}: the report and calibration are present once the state is done. */
export interface GroundResult {
  id: string
  session: string
  kind: 'ground_board'
  state: GroundResultState
  error: string | null
  created_at: string
  declined: boolean
  declined_at: string | null
  accepted_at: string | null
  /** The applied head chain the fit started from (the gauge). */
  head_chain: GroundHeadChain | null
  report: GroundReport | null
  /** The fitted head chain joints and note, null until done. */
  calibration: Record<string, unknown> | null
  /** The verification of this result once verified. */
  verification: GroundVerify | null
}

/** Answer of accept: the applied.json head_chain record and the restarted units. */
export interface GroundAccept {
  head_chain: { session: string; result: string; applied_at: string; source: string; floor: null }
  units: ArmCameraUnit[]
}

/** One entry of GET /sessions. */
export interface GroundSession {
  id: string
  created: string
  placements: number
  views: number
  planes: number
  skipped: number
  results: string[]
}

const SWEEP_STATUSES: GroundSweepStatus[] = [
  'idle',
  'starting',
  'homing',
  'locating',
  'sweeping',
  'awaiting_placement',
  'done',
  'stopped',
  'error',
]
const VERIFY_STATUSES: GroundVerifyStatus[] = ['idle', 'starting', 'waiting_for_restarts', 'measuring', 'done', 'stopped', 'error']
const RESULT_STATES: GroundResultState[] = ['running', 'done', 'failed']
const isNullStr: Check = isNullOr(isStr)
const oneOf =
  (values: readonly string[]): Check =>
  (v) =>
    values.includes(v as string)

const BOARD: Shape = { x_m: isNum, y_m: isNum, yaw_deg: isNum }
const SWEEP: Shape = {
  running: isBool,
  status: oneOf(SWEEP_STATUSES),
  session_id: isNullStr,
  placement: isNum,
  placements_done: isNum,
  max_placements: isNum,
  awaiting_placement: isBool,
  pose: isNum,
  total: isNum,
  views: isNum,
  planes: isNum,
  skipped: isNum,
  board: isNullOr(fits(BOARD)),
  message: isNullStr,
  error: isNullStr,
}
const HEAD_CHAIN: Shape = { source: isNullStr, session: isNullStr, result: isNullStr }
const RESULT_REF: Shape = { session: isStr, result: isStr }
const REACHED: Shape = { pan_deg: isNum, tilt_deg: isNum }
const VERIFY_CHECK: Shape = { name: isStr, value: isNum0, limit: isNum0, ok: isBool }
const VERIFY_POSE: Shape = {
  pose: isNum,
  pan_deg: isNum,
  tilt_deg: isNum,
  reached: isNullOr(fits(REACHED)),
  samples: isNum,
  plane_samples: isNum,
  height_m: isNum0,
  plane_vs_base_deg: isNum0,
  obstacle_points: isNum0,
  obstacle_points_max: isNum0,
  drop_points: isNum0,
  drop_points_max: isNum0,
  checks: isArrayOf(fits(VERIFY_CHECK)),
  passed: isBool,
  error: isNullStr,
}
const VERIFY: Shape = {
  status: oneOf(VERIFY_STATUSES),
  id: isNullStr,
  head_chain: isNullOr(fits(HEAD_CHAIN)),
  result: isNullOr(fits(RESULT_REF)),
  pose: isNum,
  total: isNum,
  poses: isArrayOf(fits(VERIFY_POSE)),
  passed: isNullOr(isBool),
  error: isNullStr,
}
const VERIFY_STATE: Shape = { ...VERIFY, running: isBool }
const JOB: Shape = { running: isBool, session_id: isNullStr, result_id: isNullStr, started_at: isNullStr, error: isNullStr }
const SCORE: Shape = {
  rms_px: isNum0,
  heldout_rms_px: isNum0,
  views: isNum,
  poses: isNum,
  placements: isNum,
  floor_height_rms_mm: isNum0,
  floor_normal_rms_deg: isNum0,
}
const EXAMPLE: Shape = { label: isStr, value: isNum0, unit: isStr }
const ACCEPTED: Shape = {
  session: isStr,
  result: isStr,
  accepted_at: isStr,
  score: fits(SCORE),
  accuracy_examples: isArrayOf(fits(EXAMPLE)),
  verification: isNullOr(fits(VERIFY)),
}
const STATE: Shape = {
  sweep: fits(SWEEP),
  verify: fits(VERIFY_STATE),
  compute: fits(JOB),
  head_chain: isNullOr(fits(HEAD_CHAIN)),
  accepted: isNullOr(fits(ACCEPTED)),
}
const CHECK: Shape = { name: isStr, value: isNum0, limit: isNum0, unit: isStr, message: isStr }
const PARAM: Shape = { name: isStr, value: isNum0, sigma: isNum0, unit: isStr, applied_value: isNum0, change: isNum0 }
const PLACEMENT: Shape = {
  placement: isNum,
  x_m: isNum0,
  y_m: isNum0,
  yaw_deg: isNum0,
  sigma_x_mm: isNum0,
  sigma_y_mm: isNum0,
  sigma_yaw_deg: isNum0,
  views: isNum,
  rms_px: isNum0,
}
const CURRENT_SCORE: Shape = { rms_px: isNum0, heldout_rms_px: isNum0 }
const COMPARISON: Shape = {
  applied: isNullOr(fits(HEAD_CHAIN)),
  current_score: fits(CURRENT_SCORE),
  shift_examples: isArrayOf(fits(EXAMPLE)),
}
const PLANE_SCORE: Shape = { height_rms_mm: isNum0, normal_rms_deg: isNum0 }
const CROSS_CHECK: Shape = { planes: isNum, poses: isNum, fitted: fits(PLANE_SCORE), applied: fits(PLANE_SCORE) }
const POSE: Shape = {
  pose: isNum,
  placement: isNum,
  pan_deg: isNum,
  tilt_deg: isNum,
  views: isNum,
  rms_px: isNum0,
  held_out: isBool,
}
const REPORT: Shape = {
  plausible: isBool,
  refusals: isArrayOf(fits(CHECK)),
  warnings: isArrayOf(fits(CHECK)),
  score: fits(SCORE),
  params: isArrayOf(fits(PARAM)),
  placements: isArrayOf(fits(PLACEMENT)),
  accuracy_examples: isArrayOf(fits(EXAMPLE)),
  comparison: fits(COMPARISON),
  cross_check: fits(CROSS_CHECK),
  poses: isArrayOf(fits(POSE)),
  notes: isArrayOf(isStr),
}
const RESULT: Shape = {
  id: isStr,
  session: isStr,
  kind: (k) => k === 'ground_board',
  state: oneOf(RESULT_STATES),
  error: isNullStr,
  created_at: isStr,
  declined: isBool,
  declined_at: isNullStr,
  accepted_at: isNullStr,
  head_chain: isNullOr(fits(HEAD_CHAIN)),
  report: () => true, // checked by badGroundResult with the key path
  calibration: isNullOr(isObj),
  verification: isNullOr(fits(VERIFY)),
}
const UNIT: Shape = { name: isStr, active_state: isStr, restarted: isBool }
const ACCEPT_RECORD: Shape = {
  session: isStr,
  result: isStr,
  applied_at: isStr,
  source: (s) => s === 'ground_board',
  floor: (f) => f === null,
}
const ACCEPT: Shape = { head_chain: fits(ACCEPT_RECORD), units: isArrayOf(fits(UNIT)) }
const COMPUTE: Shape = { result_id: isStr }
const SESSION: Shape = {
  id: isStr,
  created: isStr,
  placements: isNum,
  views: isNum,
  planes: isNum,
  skipped: isNum,
  results: isArrayOf(isStr),
}

/** Every shape the client checks, by name; the contract test compares their keys with the node's fixtures. */
export const GROUND_BOARD_SHAPES: Record<string, Shape> = {
  board: BOARD,
  sweep: SWEEP,
  headChain: HEAD_CHAIN,
  resultRef: RESULT_REF,
  reached: REACHED,
  verifyCheck: VERIFY_CHECK,
  verifyPose: VERIFY_POSE,
  verify: VERIFY,
  verifyState: VERIFY_STATE,
  job: JOB,
  score: SCORE,
  example: EXAMPLE,
  accepted: ACCEPTED,
  state: STATE,
  check: CHECK,
  param: PARAM,
  placement: PLACEMENT,
  currentScore: CURRENT_SCORE,
  comparison: COMPARISON,
  planeScore: PLANE_SCORE,
  crossCheck: CROSS_CHECK,
  pose: POSE,
  report: REPORT,
  result: RESULT,
  unit: UNIT,
  acceptRecord: ACCEPT_RECORD,
  accept: ACCEPT,
  compute: COMPUTE,
  session: SESSION,
}

/** Shapes nested one level under a state key, for the `key.inner` path of a state problem. */
const STATE_NESTED: Record<string, Shape> = { sweep: SWEEP, verify: VERIFY_STATE, compute: JOB, head_chain: HEAD_CHAIN, accepted: ACCEPTED }

/** First problem of a state body as a key name (`sweep.status`), or null. */
export function badGroundState(v: unknown): string | null {
  const base = badKey(v, STATE)
  if (base === null || !isObj(v) || !(base in STATE_NESTED)) return base
  return `${base}.${badKey(v[base], STATE_NESTED[base])}`
}

/** First problem of a session state body, or null. */
export const badGroundSweep = (v: unknown): string | null => badKey(v, SWEEP)

/** First problem of a verification state body, or null. */
export const badGroundVerify = (v: unknown): string | null => badKey(v, VERIFY_STATE)

/** First problem of a result body as a key name (`report.score`), or null. A done result needs its report. */
export function badGroundResult(v: unknown): string | null {
  const base = badKey(v, RESULT)
  if (base !== null || !isObj(v)) return base
  if (v.report === null) return v.state === 'done' ? 'report' : null
  const report = badKey(v.report, REPORT)
  return report === null ? null : `report.${report}`
}

/** First problem of an accept answer, or null. */
export const badGroundAccept = (v: unknown): string | null => badKey(v, ACCEPT)

/** First problem of the session list, or null. */
export const badGroundSessions = (v: unknown): string | null =>
  !Array.isArray(v) ? '(body)' : v.map((s, i) => (badKey(s, SESSION) === null ? null : `[${i}].${badKey(s, SESSION)}`)).find((k) => k !== null) ?? null

async function request<T>(url: string, init: RequestInit, bad: (v: unknown) => string | null): Promise<FloorApiResult<T>> {
  const r = await call<unknown>(url, init, () => true)
  if (!r.ok) return r
  const key = bad(r.value)
  return key === null
    ? { ok: true, value: r.value as T }
    : { ok: false, status: 200, message: `Unexpected ground board response from the robot: missing or invalid ${key}` }
}

const resultPath = (sid: string, rid: string) => `${BASE}/sessions/${seg(sid)}/results/${seg(rid)}`

export const getGroundState = () => request<GroundState>(`${BASE}/state`, GET, badGroundState)
/** Starts a ground session: the head homes, the board is located and the first placement is swept. */
export const startGroundSweep = () => request<GroundSweep>(`${BASE}/sweep/start`, POST, badGroundSweep)
/** Captures the next placement once the user moved the board. */
export const nextPlacement = () => request<GroundSweep>(`${BASE}/sweep/next-placement`, POST, badGroundSweep)
/** Ends the session (a running placement keeps its views). */
export const stopGroundSweep = () => request<GroundSweep>(`${BASE}/sweep/stop`, POST, badGroundSweep)
export const listGroundSessions = () => request<GroundSession[]>(`${BASE}/sessions`, GET, badGroundSessions)
/** Starts the fit of a session (202 {result_id}); the result is polled with getGroundResult. */
export const computeGround = (sid: string) =>
  request<{ result_id: string }>(`${BASE}/sessions/${seg(sid)}/compute`, POST, (v) => badKey(v, COMPUTE))
export const getGroundResult = (sid: string, rid: string) => request<GroundResult>(resultPath(sid, rid), GET, badGroundResult)
/** Accepts (applies the head chain of) a plausible result; a refusal (409) comes back with the node's message. */
export const acceptGround = (sid: string, rid: string) => request<GroundAccept>(`${resultPath(sid, rid)}/accept`, POST, badGroundAccept)
/** Declines a result; the answer is the result with declined true. */
export const declineGround = (sid: string, rid: string) =>
  request<GroundResult>(`${resultPath(sid, rid)}/decline`, POST, badGroundResult)
/** Starts the ground verification of the applied head chain. */
export const startGroundVerify = () => request<GroundVerifyState>(`${BASE}/verify/start`, POST, badGroundVerify)
/** Stops the verification (the head returns). */
export const stopGroundVerify = () => request<GroundVerifyState>(`${BASE}/verify/stop`, POST, badGroundVerify)
export const getGroundVerify = () => request<GroundVerifyState>(`${BASE}/verify/state`, GET, badGroundVerify)
