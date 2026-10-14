/** Fetch calls and pure helpers for the IMU calibration section (backend routes /api/calibration/imu*). */

const BASE = '/api/calibration/imu'

/** Body of GET /api/calibration/imu: BNO055 calibration levels 0-3 as the node reports them. */
export interface ImuStatus {
  sys: number
  gyro: number
  accel: number
  mag: number
  restored: boolean
  age_s?: number
}

/** Body of POST save / reset. */
export interface ImuActionResult {
  ok: boolean
  message: string
}

export type ImuApiResult<T> = { ok: true; value: T } | { ok: false; status: number; message: string }

export const IMU_LEVEL_MAX = 3

/** Theme colour of a 0-3 level: 0-1 error, 2 warning, 3 success. */
export function imuLevelColor(level: number): string {
  if (level >= IMU_LEVEL_MAX) return 'success.main'
  return level === 2 ? 'warning.main' : 'error.main'
}

/** Offsets can be saved only once the magnetometer is fully calibrated. */
export function saveEnabled(status: ImuStatus | null): boolean {
  return status !== null && status.mag === IMU_LEVEL_MAX
}

function isStatus(v: unknown): v is ImuStatus {
  const s = v as Partial<ImuStatus> | null
  return (
    !!s &&
    [s.sys, s.gyro, s.accel, s.mag].every((n) => typeof n === 'number') &&
    typeof s.restored === 'boolean'
  )
}

async function call<T>(url: string, init: RequestInit, valid: (v: unknown) => boolean): Promise<ImuApiResult<T>> {
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
    return valid(body) ? { ok: true, value: body as T } : { ok: false, status: res.status, message: 'Unexpected IMU response from the robot' }
  }
  const b = (body ?? {}) as { message?: unknown; reason?: unknown }
  const message = typeof b.message === 'string' ? b.message : typeof b.reason === 'string' ? b.reason : `HTTP ${res.status}`
  return { ok: false, status: res.status, message }
}

const isAction = (v: unknown) => typeof (v as ImuActionResult | null)?.message === 'string'

export const fetchImuStatus = () => call<ImuStatus>(BASE, { cache: 'no-store' }, isStatus)
export const postImuSave = () => call<ImuActionResult>(`${BASE}/save`, { method: 'POST' }, isAction)
export const postImuReset = () => call<ImuActionResult>(`${BASE}/reset`, { method: 'POST' }, isAction)
