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

/** One restarted consumer unit after an apply. */
export interface UnitRestart {
  name: string
  active_state?: string
  restarted: boolean
}

/** applied.json as the node writes it (`session`, `result`, `applied_at`, `files`); `units` is added by the apply answer and GET /applied. */
export interface AppliedRecord {
  session: string
  result: string
  applied_at: string
  files: string[]
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

export interface PoseSummary {
  id: string
  rgb_corners: number
  tof_corners: number
  has_depth: boolean
  /** Capture-time quality hints; absent on a node from before the hints. */
  hints?: string[]
}

/** Image-area grid; counts[row][col] is the number of stored poses covering the cell. */
export interface Coverage {
  cols: number
  rows: number
  counts: number[][]
}

/** Reprojection errors of one pose in a compute result; null where the pose has no such view. */
export interface PoseError {
  id: string
  rgb_error_px: number | null
  tof_error_px: number | null
  stereo_error_px: number | null
  outlier: string[]
}

/** result.json of one compute run. */
export interface ResultSummary {
  id: string
  created?: string | number
  passed: boolean
  /** Per-pose reprojection errors; absent on a result from before the pose errors. */
  poses?: PoseError[]
  outlier_poses?: string[]
  [key: string]: unknown
}

export interface ResultDetail extends ResultSummary {
  report: string | null
}

export interface SessionDetail {
  id: string
  created: string | number
  lens_position: number | null
  poses: PoseSummary[]
  coverage: { rgb: Coverage; tof: Coverage }
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
  isStr(v.session) &&
  isStr(v.result) &&
  isStr(v.applied_at) &&
  isArrayOf(isStr)(v.files) &&
  (v.units === undefined || isArrayOf(isUnit)(v.units))

/** applied.json is read back with `units`; the apply answer without it is rejected so the UI never shows a guess. */
const isAppliedWithUnits = (v: unknown) => isApplied(v) && isArrayOf(isUnit)((v as Record<string, unknown>).units)

const isState = (v: unknown) =>
  isObj(v) && isCapture(v.capture) && isCompute(v.compute) && isNullOr(isApplied)(v.applied ?? null)

const isSessionSummary = (v: unknown) =>
  isObj(v) && isStr(v.id) && (isStr(v.created) || isNum(v.created)) && isNum(v.pose_count) && isNum(v.result_count)

const isPose = (v: unknown) =>
  isObj(v) &&
  isStr(v.id) &&
  isNum(v.rgb_corners) &&
  isNum(v.tof_corners) &&
  typeof v.has_depth === 'boolean' &&
  (v.hints === undefined || isArrayOf(isStr)(v.hints))

const isCoverage = (v: unknown) =>
  isObj(v) && isNum(v.cols) && isNum(v.rows) && isArrayOf(isArrayOf(isNum))(v.counts)

const isPoseError = (v: unknown) =>
  isObj(v) &&
  isStr(v.id) &&
  isNullOr(isNum)(v.rgb_error_px) &&
  isNullOr(isNum)(v.tof_error_px) &&
  isNullOr(isNum)(v.stereo_error_px) &&
  isArrayOf(isStr)(v.outlier)

const isResult = (v: unknown) =>
  isObj(v) &&
  isStr(v.id) &&
  typeof v.passed === 'boolean' &&
  (v.poses === undefined || isArrayOf(isPoseError)(v.poses)) &&
  (v.outlier_poses === undefined || isArrayOf(isStr)(v.outlier_poses))

const isSession = (v: unknown) =>
  isObj(v) &&
  isStr(v.id) &&
  isNullOr(isNum)(v.lens_position ?? null) &&
  isArrayOf(isPose)(v.poses) &&
  isObj(v.coverage) &&
  isCoverage(v.coverage.rgb) &&
  isCoverage(v.coverage.tof) &&
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
export function thumbnailUrl(sid: string, pid: string, kind: 'rgb' | 'tof'): string {
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
