import { describe, expect, it } from 'vitest'
import { tiltEntered, tiltStatus, type TiltPayload } from './tiltStatus'

const WARNING: TiltPayload = { level: 'warning', roll_deg: 12.5, pitch_deg: -3 }
const ERROR: TiltPayload = { level: 'error', roll_deg: 25, pitch_deg: 4 }
const OK: TiltPayload = { level: 'ok', roll_deg: 0.5, pitch_deg: 0.2 }

describe('tiltStatus', () => {
  it('is ok without a payload: nothing blocked, no banner', () => {
    for (const p of [undefined, null, OK]) {
      expect(tiltStatus(p)).toEqual({ level: 'ok', navBlocked: false, driveBlocked: false, banner: null, navBlockedReason: null })
    }
  })

  it('warning: amber banner, goals blocked, manual drive still allowed', () => {
    const s = tiltStatus(WARNING)
    expect(s.level).toBe('warning')
    expect(s.banner).toEqual({
      severity: 'warning',
      text: 'Tilt warning: roll 12.5 / pitch -3.0 deg - automatic navigation disabled, drive back manually',
    })
    expect(s.navBlocked).toBe(true)
    expect(s.driveBlocked).toBe(false)
    expect(s.navBlockedReason).toBe(s.banner?.text)
  })

  it('error: red banner, goals and manual drive blocked', () => {
    const s = tiltStatus(ERROR)
    expect(s.banner).toEqual({ severity: 'error', text: 'Tilt error: base drive disabled until the robot is level' })
    expect(s.navBlocked).toBe(true)
    expect(s.driveBlocked).toBe(true)
  })

  it('warning + base_drive_inhibited -> driveBlocked with the still-inhibited text', () => {
    const s = tiltStatus({ ...WARNING, base_drive_inhibited: true })
    expect(s.level).toBe('warning')
    expect(s.driveBlocked).toBe(true)
    expect(s.navBlocked).toBe(true)
    expect(s.banner?.severity).toBe('warning')
    expect(s.banner?.text).toBe(
      'Tilt warning: roll 12.5 / pitch -3.0 deg - base drive still inhibited after the error, wait until the robot is level',
    )
    expect(s.navBlockedReason).toBe(s.banner?.text)
    expect(tiltStatus({ ...WARNING, base_drive_inhibited: false }).driveBlocked).toBe(false)
  })

  it('error stays driveBlocked whatever base_drive_inhibited says', () => {
    expect(tiltStatus({ ...ERROR, base_drive_inhibited: false }).driveBlocked).toBe(true)
  })

  it('stale state: goals blocked with a stale banner, even at ok', () => {
    const s = tiltStatus({ ...OK, stale: true })
    expect(s.navBlocked).toBe(true)
    expect(s.driveBlocked).toBe(false)
    expect(s.banner).toEqual({ severity: 'warning', text: 'Tilt guard state is stale (no recent update) - navigation disabled until it reports again' })
    expect(s.navBlockedReason).toBe(s.banner?.text)
  })

  it('imu_ok false at ok: goals blocked with an IMU banner', () => {
    const s = tiltStatus({ ...OK, imu_ok: false })
    expect(s.navBlocked).toBe(true)
    expect(s.banner?.text).toBe('Tilt guard IMU not OK - navigation disabled until the IMU reports again')
    expect(tiltStatus({ ...OK, imu_ok: true }).navBlocked).toBe(false)
  })

  it('shows ? for missing angles and ignores unknown levels', () => {
    expect(tiltStatus({ level: 'warning' }).banner?.text).toContain('roll ? / pitch ? deg')
    expect(tiltStatus({ level: 'bogus' } as unknown as TiltPayload).level).toBe('ok')
  })
})

describe('tiltEntered', () => {
  it('fires on entering or worsening, not on repeats, relief or ok', () => {
    expect(tiltEntered('ok', 'warning')).toBe(true)
    expect(tiltEntered('ok', 'error')).toBe(true)
    expect(tiltEntered('warning', 'error')).toBe(true)
    expect(tiltEntered('warning', 'warning')).toBe(false)
    expect(tiltEntered('error', 'warning')).toBe(false)
    expect(tiltEntered('warning', 'ok')).toBe(false)
    expect(tiltEntered('ok', 'ok')).toBe(false)
  })
})
