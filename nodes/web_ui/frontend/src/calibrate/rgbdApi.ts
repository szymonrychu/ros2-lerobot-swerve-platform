/** Typed fetch calls and pure helpers for the RGB-D calibration section (backend routes /api/calibration/rgbd/*). */

const BASE = '/api/calibration/rgbd'

/** Live capture state of the calibration node. */
export interface CaptureState {
  running: boolean
  session_id: string | null
  status: string
  pose_count: number
  started_at: string | null
  stops_at: string | null
}

/** Compute job state of the calibration node. */
export interface ComputeState {
  running: boolean
  session_id: string | null
  result_id: string | null
  started_at: string | null
  error: string | null
}

/** State of one consumer unit, as the node reports it next to applied.json (an intrinsics accept restarts none). */
export interface UnitRestart {
  name: string
  active_state?: string
  restarted: boolean
}

/** The accepted D435 intrinsics check in applied.json (`kind`, `session`, `result`, `applied_at`); `units` is added by the apply answer and GET /applied. */
export interface AppliedRecord {
  kind: string
  session: string
  result: string
  applied_at: string
  units?: UnitRestart[]
}

/** Body of GET /applied and the answer of apply: applied.json plus the unit restarts. */
export interface AppliedState extends AppliedRecord {
  units: UnitRestart[]
}

/** Body of GET /state; `applied` is null while nothing is applied. */
export interface RgbdState {
  capture: CaptureState
  compute: ComputeState
  applied: AppliedRecord | null
}

export interface SessionSummary {
  id: string
  created: string | number
  pose_count: number
  result_count: number
}

/** One stored still of a D435 intrinsics session. */
export interface PoseSummary {
  id: string
  /** ChArUco corners detected in the colour image. */
  rgb_corners: number
  /** Aligned depth pixels with a value inside the detected board corners (the plane bias samples). */
  plane_bias_samples: number
  /** Capture-time quality hints. */
  hints: string[]
}

/** Image-area grid; counts[row][col] is the number of stored poses covering the cell. */
export interface Coverage {
  cols: number
  rows: number
  counts: number[][]
}

/** Colour reprojection error and board-plane depth bias of one still in a check result; null where not measured. */
export interface PoseError {
  id: string
  error_px: number | null
  plane_bias_mm: number | null
  plane_spread_mm: number | null
}

/** Acceptance limits recorded in result.json (`limits`, the node's D435 limit keys). */
export interface CheckLimits {
  plane_max_bias_mm: number
  plane_max_spread_mm: number
  min_views: number
  intrinsics_max_rms_px: number
  intrinsics_max_focal_diff_pct: number
  intrinsics_max_principal_diff_px: number
}

/** result.json of one D435 intrinsics check; metrics are null where an input error stopped the check. */
export interface ResultSummary {
  id: string
  session: string
  created: string
  kind: 'intrinsics'
  passed: boolean
  /** One description per exceeded limit, each starting with its `limits.<key>:`. */
  failed_limits: string[]
  rms_px: number | null
  focal_diff_pct: number | null
  principal_diff_px: number | null
  plane_bias_median_mm: number | null
  plane_bias_spread_mm: number | null
  views: number | null
  pose_ids: string[]
  poses: PoseError[]
  limits: CheckLimits
  /** Input error (too few views, no factory camera_info, ...); set only when the check could not run. */
  error?: string
}

export interface ResultDetail extends ResultSummary {
  report: string | null
}

export interface SessionDetail {
  id: string
  created: string | number
  poses: PoseSummary[]
  coverage: { rgb: Coverage }
  results: ResultSummary[]
}

export type RgbdApiResult<T> = { ok: true; value: T } | { ok: false; status: number; message: string }
export type StatusSeverity = 'success' | 'info' | 'warning'

const isObj = (v: unknown): v is Record<string, unknown> => typeof v === 'object' && v !== null && !Array.isArray(v)
const isStr = (v: unknown) => typeof v === 'string'
const isNum = (v: unknown) => typeof v === 'number'
const isNullOr = (f: (v: unknown) => boolean) => (v: unknown) => v === null || f(v)
const isArrayOf = (f: (v: unknown) => boolean) => (v: unknown) => Array.isArray(v) && v.every(f)

function isCapture(v: unknown): boolean {
  return (
    isObj(v) &&
    typeof v.running === 'boolean' &&
    isNullOr(isStr)(v.session_id) &&
    isStr(v.status) &&
    isNum(v.pose_count) &&
    isNullOr(isStr)(v.started_at) &&
    isNullOr(isStr)(v.stops_at)
  )
}

function isCompute(v: unknown): boolean {
  return (
    isObj(v) &&
    typeof v.running === 'boolean' &&
    isNullOr(isStr)(v.session_id) &&
    isNullOr(isStr)(v.result_id) &&
    isNullOr(isStr)(v.started_at) &&
    isNullOr(isStr)(v.error)
  )
}

const isUnit = (v: unknown) => isObj(v) && isStr(v.name) && typeof v.restarted === 'boolean'
const isApplied = (v: unknown) =>
  isObj(v) &&
  isStr(v.kind) &&
  isStr(v.session) &&
  isStr(v.result) &&
  isStr(v.applied_at) &&
  (v.units === undefined || isArrayOf(isUnit)(v.units))

/** The apply answer and GET /applied carry the node's `units` next to applied.json. */
const isAppliedWithUnits = (v: unknown) => isApplied(v) && isArrayOf(isUnit)((v as Record<string, unknown>).units)

const isState = (v: unknown) =>
  isObj(v) && isCapture(v.capture) && isCompute(v.compute) && isNullOr(isApplied)(v.applied ?? null)

const isSessionSummary = (v: unknown) =>
  isObj(v) && isStr(v.id) && (isStr(v.created) || isNum(v.created)) && isNum(v.pose_count) && isNum(v.result_count)

const isPose = (v: unknown) =>
  isObj(v) && isStr(v.id) && isNum(v.rgb_corners) && isNum(v.plane_bias_samples) && isArrayOf(isStr)(v.hints)

const isCoverage = (v: unknown) =>
  isObj(v) && isNum(v.cols) && isNum(v.rows) && isArrayOf(isArrayOf(isNum))(v.counts)

const isPoseError = (v: unknown) =>
  isObj(v) &&
  isStr(v.id) &&
  isNullOr(isNum)(v.error_px) &&
  isNullOr(isNum)(v.plane_bias_mm) &&
  isNullOr(isNum)(v.plane_spread_mm)

const LIMIT_KEYS: (keyof CheckLimits)[] = [
  'plane_max_bias_mm',
  'plane_max_spread_mm',
  'min_views',
  'intrinsics_max_rms_px',
  'intrinsics_max_focal_diff_pct',
  'intrinsics_max_principal_diff_px',
]
const METRIC_KEYS = [
  'rms_px',
  'focal_diff_pct',
  'principal_diff_px',
  'plane_bias_median_mm',
  'plane_bias_spread_mm',
  'views',
] as const

const isLimits = (v: unknown) => isObj(v) && LIMIT_KEYS.every((k) => isNum(v[k]))

const isResult = (v: unknown) =>
  isObj(v) &&
  isStr(v.id) &&
  isStr(v.session) &&
  isStr(v.created) &&
  v.kind === 'intrinsics' &&
  typeof v.passed === 'boolean' &&
  isArrayOf(isStr)(v.failed_limits) &&
  METRIC_KEYS.every((k) => isNullOr(isNum)(v[k])) &&
  isArrayOf(isStr)(v.pose_ids) &&
  isArrayOf(isPoseError)(v.poses) &&
  isLimits(v.limits) &&
  (v.error === undefined || isStr(v.error))

const isSession = (v: unknown) =>
  isObj(v) &&
  isStr(v.id) &&
  isArrayOf(isPose)(v.poses) &&
  isObj(v.coverage) &&
  isCoverage(v.coverage.rgb) &&
  isArrayOf(isResult)(v.results)

const isResultDetail = (v: unknown) => isResult(v) && isNullOr(isStr)((v as Record<string, unknown>).report)
const isComputeStarted = (v: unknown) => isObj(v) && isStr(v.result_id)
const isAnyObject = (v: unknown) => isObj(v)

/** Number of grid cells no stored pose covers. */
export function emptyCells(coverage: Coverage): number {
  return coverage.counts.reduce((sum, row) => sum + row.filter((n) => n === 0).length, 0)
}

/** Pose errors of the newest result (`results[0]`, the node lists newest first) keyed by pose id; empty without one. */
export function poseErrors(detail: SessionDetail): Map<string, PoseError> {
  return new Map((detail.results[0]?.poses ?? []).map((p) => [p.id, p]))
}

/** Alert severity of a capture status line: captured is success, the three `still - ...` messages warn. */
export function statusSeverity(status: string): StatusSeverity {
  if (status.includes('captured')) return 'success'
  if (status.startsWith('still')) return 'warning'
  return 'info'
}

const seg = encodeURIComponent
const sessionPath = (sid: string) => `${BASE}/sessions/${seg(sid)}`

/** URL of the stored thumbnail (corners drawn) of one pose. */
export function thumbnailUrl(sid: string, pid: string, kind: 'rgb' | 'depth'): string {
  return `${sessionPath(sid)}/poses/${seg(pid)}/${kind}.jpg`
}

/** URL of the zip download of one result. */
export function downloadUrl(sid: string, rid: string): string {
  return `${sessionPath(sid)}/results/${seg(rid)}/download`
}

async function call<T>(url: string, init: RequestInit, valid: (v: unknown) => boolean): Promise<RgbdApiResult<T>> {
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
    return valid(body)
      ? { ok: true, value: body as T }
      : { ok: false, status: res.status, message: 'Unexpected calibration response from the robot' }
  }
  const b = (body ?? {}) as { message?: unknown; reason?: unknown }
  const message = typeof b.message === 'string' ? b.message : typeof b.reason === 'string' ? b.reason : `HTTP ${res.status}`
  return { ok: false, status: res.status, message }
}

const GET: RequestInit = { cache: 'no-store' }

export const getState = () => call<RgbdState>(`${BASE}/state`, GET, isState)

/** Start a capture, on a new session or (with sessionId) on an existing one. */
export const startCapture = (sessionId?: string) =>
  call<RgbdState>(
    `${BASE}/capture/start`,
    {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(sessionId === undefined ? {} : { session_id: sessionId }),
    },
    isState,
  )
export const stopCapture = () => call<RgbdState>(`${BASE}/capture/stop`, { method: 'POST' }, isState)
export const listSessions = () => call<SessionSummary[]>(`${BASE}/sessions`, GET, isArrayOf(isSessionSummary))
export const getSession = (sid: string) => call<SessionDetail>(sessionPath(sid), GET, isSession)
export const deletePose = (sid: string, pid: string) =>
  call<Record<string, unknown>>(`${sessionPath(sid)}/poses/${seg(pid)}`, { method: 'DELETE' }, isAnyObject)
export const computeSession = (sid: string) =>
  call<{ result_id: string }>(`${sessionPath(sid)}/compute`, { method: 'POST' }, isComputeStarted)
export const getResult = (sid: string, rid: string) =>
  call<ResultDetail>(`${sessionPath(sid)}/results/${seg(rid)}`, GET, isResultDetail)
export const applyResult = (sid: string, rid: string) =>
  call<AppliedState>(`${sessionPath(sid)}/results/${seg(rid)}/apply`, { method: 'POST' }, isAppliedWithUnits)
export const getApplied = () => call<AppliedState>(`${BASE}/applied`, GET, isAppliedWithUnits)
