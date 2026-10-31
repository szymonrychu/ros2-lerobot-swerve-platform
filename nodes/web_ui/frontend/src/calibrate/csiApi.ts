/** Typed fetch calls for the CSI camera intrinsics section (node routes /api/csi/{camera}/* behind
 * /api/calibration/rgbd/csi/{camera}/*). */

import type { Coverage, UnitRestart } from './rgbdApi'

const BASE = '/api/calibration/rgbd/csi'

/** The CSI cameras of the robot (head_rgbd_calibration `csi_cameras`), in selector order. */
export const CSI_CAMERAS = ['front', 'back'] as const
export type CsiCamera = (typeof CSI_CAMERAS)[number]

/** The latest CSI capture of any camera: kept views and the status line. */
export interface CsiCapture {
  running: boolean
  camera: string | null
  session_id: string | null
  status: string
  view_count: number
  started_at: string | null
  stops_at: string | null
}

/** result.json of one CSI intrinsics compute: K (3x3), plumb_bob D, RMS (px) and per-view errors. */
export interface CsiResult {
  kind: 'csi_intrinsics'
  camera: string
  session: string
  result: string
  created: string
  image_size: number[]
  k: number[][]
  d: number[]
  rms: number
  per_view_errors: number[]
  view_count: number
}

/** The capture's session when it is this camera's, else the camera's newest one. */
export interface CsiSession {
  id: string
  created: string
  view_count: number
  image_size: number[] | null
  coverage: Coverage
  results: CsiResult[]
}

/** The camera's record in applied.json. */
export interface CsiApplied {
  kind: 'csi_intrinsics'
  camera: string
  session: string
  result: string
  applied_at: string
}

/** Body of GET /csi/{camera}/state and the answer of start and stop. */
export interface CsiState {
  camera: string
  capture: CsiCapture
  session: CsiSession | null
  applied: CsiApplied | null
}

/** Answer of apply: the record and the camera unit's restart state. */
export interface CsiApplyAnswer {
  applied: CsiApplied
  units: UnitRestart[]
}

export type CsiApiResult<T> = { ok: true; value: T } | { ok: false; status: number; message: string }

const isObj = (v: unknown): v is Record<string, unknown> => typeof v === 'object' && v !== null && !Array.isArray(v)
const isStr = (v: unknown) => typeof v === 'string'
const isNum = (v: unknown) => typeof v === 'number'
const isNullOr = (f: (v: unknown) => boolean) => (v: unknown) => v === null || f(v)
const isArrayOf = (f: (v: unknown) => boolean) => (v: unknown) => Array.isArray(v) && v.every(f)

const isCapture = (v: unknown) =>
  isObj(v) &&
  typeof v.running === 'boolean' &&
  isNullOr(isStr)(v.camera) &&
  isNullOr(isStr)(v.session_id) &&
  isStr(v.status) &&
  isNum(v.view_count) &&
  isNullOr(isStr)(v.started_at) &&
  isNullOr(isStr)(v.stops_at)

const isMatrix3 = (v: unknown) => Array.isArray(v) && v.length === 3 && v.every((row) => isArrayOf(isNum)(row) && row.length === 3)

const isResult = (v: unknown) =>
  isObj(v) &&
  v.kind === 'csi_intrinsics' &&
  isStr(v.camera) &&
  isStr(v.session) &&
  isStr(v.result) &&
  isStr(v.created) &&
  isArrayOf(isNum)(v.image_size) &&
  isMatrix3(v.k) &&
  isArrayOf(isNum)(v.d) &&
  isNum(v.rms) &&
  isArrayOf(isNum)(v.per_view_errors) &&
  isNum(v.view_count)

const isCoverage = (v: unknown) => isObj(v) && isNum(v.cols) && isNum(v.rows) && isArrayOf(isArrayOf(isNum))(v.counts)

const isSession = (v: unknown) =>
  isObj(v) &&
  isStr(v.id) &&
  isStr(v.created) &&
  isNum(v.view_count) &&
  isNullOr(isArrayOf(isNum))(v.image_size) &&
  isCoverage(v.coverage) &&
  isArrayOf(isResult)(v.results)

const isApplied = (v: unknown) =>
  isObj(v) && v.kind === 'csi_intrinsics' && isStr(v.camera) && isStr(v.session) && isStr(v.result) && isStr(v.applied_at)

const isUnit = (v: unknown) => isObj(v) && isStr(v.name) && typeof v.restarted === 'boolean'

const isState = (v: unknown) =>
  isObj(v) && isStr(v.camera) && isCapture(v.capture) && isNullOr(isSession)(v.session) && isNullOr(isApplied)(v.applied)

const isApplyAnswer = (v: unknown) => isObj(v) && isApplied(v.applied) && isArrayOf(isUnit)(v.units)

async function call<T>(url: string, init: RequestInit, valid: (v: unknown) => boolean): Promise<CsiApiResult<T>> {
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
      : { ok: false, status: res.status, message: 'Unexpected CSI calibration response from the robot' }
  }
  const b = (body ?? {}) as { message?: unknown }
  return { ok: false, status: res.status, message: typeof b.message === 'string' ? b.message : `HTTP ${res.status}` }
}

const GET: RequestInit = { cache: 'no-store' }
const POST: RequestInit = { method: 'POST' }
const seg = encodeURIComponent
const cameraPath = (camera: string) => `${BASE}/${seg(camera)}`
const sessionPath = (camera: string, sid: string) => `${cameraPath(camera)}/sessions/${seg(sid)}`

export const getCsiState = (camera: string) => call<CsiState>(`${cameraPath(camera)}/state`, GET, isState)

/** Start a capture of the camera, on a new session or (with sessionId) on an existing one. */
export const startCsiCapture = (camera: string, sessionId?: string) =>
  call<CsiState>(
    `${cameraPath(camera)}/start`,
    {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(sessionId === undefined ? {} : { session_id: sessionId }),
    },
    isState,
  )
export const stopCsiCapture = (camera: string) => call<CsiState>(`${cameraPath(camera)}/stop`, POST, isState)
export const computeCsi = (camera: string, sid: string) =>
  call<CsiResult>(`${sessionPath(camera, sid)}/compute`, POST, isResult)
export const listCsiResults = (camera: string, sid: string) =>
  call<CsiResult[]>(`${sessionPath(camera, sid)}/results`, GET, isArrayOf(isResult))
export const getCsiResult = (camera: string, sid: string, rid: string) =>
  call<CsiResult>(`${sessionPath(camera, sid)}/results/${seg(rid)}`, GET, isResult)
export const applyCsi = (camera: string, sid: string, rid: string) =>
  call<CsiApplyAnswer>(`${sessionPath(camera, sid)}/results/${seg(rid)}/apply`, POST, isApplyAnswer)
