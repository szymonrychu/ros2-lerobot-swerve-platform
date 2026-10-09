import { describe, expect, it } from 'vitest'
import {
  baseGpsStatus,
  roverGpsStatus,
  type BaseGpsSample,
  type GpsStatusConfig,
  type RoverGpsPayload,
} from './gpsStatus'

const CFG: GpsStatusConfig = {
  rover_topic: '/client/gps/status',
  base_url: 'http://server:18100/topics/server/gps/status',
  base_poll_hz: 1,
  stale_after_s: 5,
  base_timeout_s: 2,
}
const NOW = 100_000

function rover(quality: number, over: Partial<RoverGpsPayload> = {}): RoverGpsPayload {
  const labels: Record<number, string> = { 0: 'No fix', 1: 'GPS', 2: 'DGPS', 4: 'RTK Fixed', 5: 'RTK Float', 6: 'Dead reckoning' }
  return {
    role: 'rover',
    quality,
    fix: labels[quality] ?? 'Unknown',
    num_satellites: 18,
    hdop: 0.7,
    diff_age_s: 1,
    ntrip_connected: true,
    ntrip_rx_bytes: 1234,
    ...over,
  }
}

function base(quality: number, over: Partial<BaseGpsSample> = {}): BaseGpsSample {
  return {
    reachable: true,
    role: 'base',
    quality,
    fix: quality === 4 ? 'RTK Fixed' : 'GPS',
    num_satellites: 22,
    hdop: 0.6,
    ntrip_clients: 1,
    rtcm_tx_frames: 10,
    rtcm_tx_bytes: 5000,
    rtcm_types: [1005],
    stale: false,
    ...over,
  }
}

describe('roverGpsStatus', () => {
  it('colours every quality', () => {
    expect(roverGpsStatus(rover(4), CFG, NOW, NOW)).toMatchObject({ level: 'rtk_fixed', color: 'success' })
    expect(roverGpsStatus(rover(5), CFG, NOW, NOW)).toMatchObject({ level: 'rtk_float', color: 'warning' })
    expect(roverGpsStatus(rover(2), CFG, NOW, NOW)).toMatchObject({ level: 'dgps', color: 'warning' })
    expect(roverGpsStatus(rover(1), CFG, NOW, NOW)).toMatchObject({ level: 'gps', color: 'info' })
    expect(roverGpsStatus(rover(0), CFG, NOW, NOW)).toMatchObject({ level: 'no_fix', color: 'error' })
    expect(roverGpsStatus(rover(6), CFG, NOW, NOW)).toMatchObject({ level: 'other', color: 'default' })
  })

  it('shows fix and satellites', () => {
    expect(roverGpsStatus(rover(4), CFG, NOW, NOW).label).toBe('Robot: RTK Fixed - 18 sats')
    expect(roverGpsStatus(rover(4), CFG, NOW, NOW).shortLabel).toBe('Robot: RTK Fixed')
    expect(roverGpsStatus(rover(4, { num_satellites: null }), CFG, NOW, NOW).label).toBe('Robot: RTK Fixed')
  })

  it('is no data before the first sample or without config', () => {
    expect(roverGpsStatus(undefined, CFG, NOW, null)).toMatchObject({ level: 'no_data', color: 'default', label: 'Robot: no data' })
    expect(roverGpsStatus(rover(4), CFG, NOW, null).level).toBe('no_data')
    expect(roverGpsStatus(rover(4), null, NOW, NOW).level).toBe('no_data')
  })

  it('is stale (red) when no message within stale_after_s', () => {
    expect(roverGpsStatus(rover(4), CFG, NOW + 5000, NOW).level).toBe('rtk_fixed')
    expect(roverGpsStatus(rover(4), CFG, NOW + 5001, NOW)).toMatchObject({ level: 'stale', color: 'error', label: 'Robot: stale' })
  })

  it('puts details in the tooltip', () => {
    const tip = roverGpsStatus(rover(4), CFG, NOW, NOW).tooltip
    expect(tip).toContain('HDOP 0.7')
    expect(tip).toContain('NTRIP connected')
    expect(tip).toContain('NTRIP rx 1234 B')
  })
})

describe('baseGpsStatus', () => {
  const r = rover(4)

  it('colours every quality', () => {
    expect(baseGpsStatus(base(4), r, CFG, NOW, NOW, NOW)).toMatchObject({ level: 'rtk_fixed', color: 'success' })
    expect(baseGpsStatus(base(5), r, CFG, NOW, NOW, NOW)).toMatchObject({ level: 'rtk_float', color: 'warning' })
    expect(baseGpsStatus(base(2), r, CFG, NOW, NOW, NOW)).toMatchObject({ level: 'dgps', color: 'warning' })
    expect(baseGpsStatus(base(1), r, CFG, NOW, NOW, NOW)).toMatchObject({ level: 'gps', color: 'info' })
    expect(baseGpsStatus(base(0), r, CFG, NOW, NOW, NOW)).toMatchObject({ level: 'no_fix', color: 'error' })
  })

  it('shows fix, sats and link OK with differential age', () => {
    expect(baseGpsStatus(base(4), r, CFG, NOW, NOW, NOW).label).toBe('Base: RTK Fixed - 22 sats - link OK 1.0s')
    expect(baseGpsStatus(base(4), rover(4, { diff_age_s: null }), CFG, NOW, NOW, NOW).label).toBe(
      'Base: RTK Fixed - 22 sats - link OK',
    )
  })

  it('says link down (red) when the rover reports ntrip_connected false', () => {
    const s = baseGpsStatus(base(4), rover(4, { ntrip_connected: false }), CFG, NOW, NOW, NOW)
    expect(s.label).toBe('Base: RTK Fixed - 22 sats - link down')
    expect(s.color).toBe('error')
  })

  it('omits the link when the rover status is unknown or stale', () => {
    expect(baseGpsStatus(base(4), undefined, CFG, NOW, NOW, null).label).toBe('Base: RTK Fixed - 22 sats')
    expect(baseGpsStatus(base(4), r, CFG, NOW, NOW, NOW - 6000).label).toBe('Base: RTK Fixed - 22 sats')
  })

  it('is "base unreachable" (red) when the poll failed, with the reason in the tooltip', () => {
    const s = baseGpsStatus({ reachable: false, error: 'timeout' }, r, CFG, NOW, NOW, NOW)
    expect(s).toMatchObject({ level: 'unreachable', color: 'error', label: 'Base: unreachable' })
    expect(s.tooltip).toContain('timeout')
  })

  it('is no data before the first poll', () => {
    expect(baseGpsStatus(undefined, r, CFG, NOW, null, NOW)).toMatchObject({ level: 'no_data', label: 'Base: no data' })
  })

  it('is stale when the backend flags it or no poll result arrived within stale_after_s', () => {
    expect(baseGpsStatus(base(4, { stale: true }), r, CFG, NOW, NOW, NOW)).toMatchObject({ level: 'stale', color: 'error' })
    expect(baseGpsStatus(base(4), r, CFG, NOW + 5001, NOW, NOW).level).toBe('stale')
  })

  it('puts details in the tooltip', () => {
    const tip = baseGpsStatus(base(4), r, CFG, NOW, NOW, NOW).tooltip
    expect(tip).toContain('HDOP 0.6')
    expect(tip).toContain('RTCM tx 5000 B')
    expect(tip).toContain('1 NTRIP client')
    expect(tip).toContain('NTRIP rx 1234 B')
  })
})
