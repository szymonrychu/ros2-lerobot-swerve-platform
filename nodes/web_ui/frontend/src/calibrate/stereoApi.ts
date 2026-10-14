/** Fetch calls and pure helpers for the stereo calibration section (backend proxy to the Jetson depthd). */

const BASE = '/api/calibration/stereo'

export type Side = 'left' | 'right'
export const SIDES: readonly Side[] = ['left', 'right']

export interface SideDetection {
  corners: number
  markers?: number
}

/** Body of GET /preview. */
export interface PreviewInfo {
  seq: number
  left: SideDetection
  right: SideDetection
  shared_corners: number
  min_shared_corners: number
  capturable: boolean
  captures: number
  min_pairs_to_solve: number
}

export interface CaptureEntry {
  id: string
}

/** Body of GET /captures. */
export interface CaptureList {
  captures: CaptureEntry[]
  count: number
  min_pairs_to_solve: number
}

export interface CaptureResult {
  id: string
  captures: number
}

export interface DeleteResult {
  removed_captures: number
  calibration_removed: boolean
}

/** Body of GET/POST /solve; numbers are null or absent when the job did not produce them. */
export interface SolveStatus {
  state: string
  rms?: number | null
  rms_left?: number | null
  rms_right?: number | null
  pairs_used?: number | null
  pairs_skipped?: number | null
  message?: string | null
}

export type ApiResult<T> = { ok: true; value: T } | { ok: false; status: number; message: string }

const FINAL_SOLVE_STATES: readonly string[] = ['written', 'refused', 'failed', 'timeout']

/** Theme colour of the shared-corner count: ok when a pair is capturable. */
export function capturableColor(capturable: boolean): string {
  return capturable ? 'success.main' : 'error.main'
}

/** Preview image URL pinned to one seq so both sides show the same pair. */
export function previewImageUrl(side: Side, seq: number): string {
  return `${BASE}/preview/${side}.jpg?seq=${seq}`
}

export function captureThumbUrl(id: string, side: Side): string {
  return `${BASE}/captures/${encodeURIComponent(id)}/${side}.jpg`
}

/** Whether a solve job state is terminal (polling stops). */
export function isFinalSolveState(state: string): boolean {
  return FINAL_SOLVE_STATES.includes(state)
}

async function call<T>(url: string, init: RequestInit): Promise<ApiResult<T>> {
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
  if (res.ok) return { ok: true, value: body as T }
  const b = (body ?? {}) as { reason?: unknown; message?: unknown }
  const message = typeof b.reason === 'string' ? b.reason : typeof b.message === 'string' ? b.message : `HTTP ${res.status}`
  return { ok: false, status: res.status, message }
}

export async function fetchPreview(): Promise<ApiResult<PreviewInfo>> {
  const r = await call<PreviewInfo>(`${BASE}/preview`, { cache: 'no-store' })
  if (r.ok && (typeof r.value?.seq !== 'number' || !r.value.left || !r.value.right)) {
    return { ok: false, status: 200, message: 'Unexpected preview response from the robot' }
  }
  return r
}
export const postCapture = () => call<CaptureResult>(`${BASE}/captures`, { method: 'POST' })
export const fetchCaptures = () => call<CaptureList>(`${BASE}/captures`, { cache: 'no-store' })
export const deleteCaptures = () => call<DeleteResult>(`${BASE}/captures?confirm=true`, { method: 'DELETE' })
export const postSolve = () => call<SolveStatus>(`${BASE}/solve`, { method: 'POST' })
export const fetchSolve = () => call<SolveStatus>(`${BASE}/solve`, { cache: 'no-store' })
