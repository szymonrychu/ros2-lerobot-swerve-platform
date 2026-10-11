/** Pure tilt guard logic: banner text, what is blocked and when to notify, from the /tilt_guard/state payload. */

/** Topic (and WS key) carrying the tilt guard state, published at 1 Hz and latched. */
export const TILT_GUARD_STATE_KEY = '/tilt_guard/state'

export type TiltLevel = 'ok' | 'warning' | 'error'

/** Tilt guard state payload (JSON of std_msgs/String, forwarded as is). */
export interface TiltPayload {
  level: TiltLevel
  roll_deg?: number | null
  pitch_deg?: number | null
  /** The swerve controller drops every cmd_vel: latched from an error until the level is back at ok. */
  base_drive_inhibited?: boolean
  /** False while the guard's IMU is dead (the level then stays at its last value). */
  imu_ok?: boolean
  /** Added by the web_ui bridge when no state arrived for tilt_guard_stale_s. */
  stale?: boolean
}

export interface TiltBannerInfo {
  severity: 'warning' | 'error'
  text: string
}

export interface TiltStatus {
  level: TiltLevel
  /** Automatic navigation (map click / goal buttons) is disabled: warning and error. */
  navBlocked: boolean
  /** Manual base drive is disabled: error only. */
  driveBlocked: boolean
  banner: TiltBannerInfo | null
  /** Tooltip explaining why goals are disabled, or null. */
  navBlockedReason: string | null
}

const SEVERITY: Record<TiltLevel, number> = { ok: 0, warning: 1, error: 2 }

const OK_STATUS: TiltStatus = { level: 'ok', navBlocked: false, driveBlocked: false, banner: null, navBlockedReason: null }

function angle(deg: number | null | undefined): string {
  return typeof deg === 'number' && Number.isFinite(deg) ? deg.toFixed(1) : '?'
}

const STALE_TEXT = 'Tilt guard state is stale (no recent update) - navigation disabled until it reports again'
const IMU_TEXT = 'Tilt guard IMU not OK - navigation disabled until the IMU reports again'

/** Derive the UI status from the payload; no payload or an unknown level means ok, a stale state or dead IMU blocks goals. */
export function tiltStatus(payload: TiltPayload | null | undefined): TiltStatus {
  if (!payload) return OK_STATUS
  if (payload.level === 'error') {
    const text = 'Tilt error: base drive disabled until the robot is level'
    return { level: 'error', navBlocked: true, driveBlocked: true, banner: { severity: 'error', text }, navBlockedReason: text }
  }
  if (payload.level === 'warning') {
    const inhibited = payload.base_drive_inhibited === true
    const reading = `Tilt warning: roll ${angle(payload.roll_deg)} / pitch ${angle(payload.pitch_deg)} deg`
    const text = inhibited
      ? `${reading} - base drive still inhibited after the error, wait until the robot is level`
      : `${reading} - automatic navigation disabled, drive back manually`
    return { level: 'warning', navBlocked: true, driveBlocked: inhibited, banner: { severity: 'warning', text }, navBlockedReason: text }
  }
  if (payload.level !== 'ok') return OK_STATUS
  const text = payload.stale === true ? STALE_TEXT : payload.imu_ok === false ? IMU_TEXT : null
  if (text === null) return OK_STATUS
  return { level: 'ok', navBlocked: true, driveBlocked: false, banner: { severity: 'warning', text }, navBlockedReason: text }
}

/** True when the level just rose to warning or error (the moment for an interrupting toast). */
export function tiltEntered(previous: TiltLevel, next: TiltLevel): boolean {
  return next !== 'ok' && SEVERITY[next] > SEVERITY[previous]
}
