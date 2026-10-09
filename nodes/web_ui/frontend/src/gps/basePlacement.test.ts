import { describe, expect, it } from 'vitest'
import { GpsAnchor, latLonToMap, mapToLatLon } from '../map3d/geo'
import { baseMarkerLabel, basePlacement } from './basePlacement'
import type { BaseGpsPayload, BaseGpsSample } from './gpsStatus'

const ANCHOR: GpsAnchor = { lat: 52.0, lon: 21.0, heading_rad: 0.6 }
const BASE: BaseGpsSample = {
  reachable: true,
  role: 'base',
  quality: 4,
  fix: 'RTK Fixed',
  num_satellites: 18,
  hdop: 0.7,
  ntrip_clients: 1,
  rtcm_tx_frames: 1,
  rtcm_tx_bytes: 1,
  rtcm_types: [1005],
  latitude: 52.0003,
  longitude: 21.0004,
}

describe('basePlacement', () => {
  it('places the base with the same maths as latLonToMap', () => {
    const p = basePlacement(ANCHOR, BASE)!
    const q = latLonToMap(ANCHOR, 52.0003, 21.0004)
    expect(p.x).toBeCloseTo(q.x, 9)
    expect(p.y).toBeCloseTo(q.y, 9)
  })

  it('round-trips back to the base lat/lon', () => {
    const p = basePlacement(ANCHOR, BASE)!
    const ll = mapToLatLon(ANCHOR, p.x, p.y)
    expect(ll.latitude).toBeCloseTo(52.0003, 9)
    expect(ll.longitude).toBeCloseTo(21.0004, 9)
  })

  it('puts a base at the anchor position at the map origin', () => {
    const p = basePlacement(ANCHOR, { ...BASE, latitude: 52.0, longitude: 21.0 })!
    expect(Math.hypot(p.x, p.y)).toBeCloseTo(0, 9)
  })

  it('is null without an anchor or status', () => {
    expect(basePlacement(null, BASE)).toBeNull()
    expect(basePlacement(ANCHOR, null)).toBeNull()
    expect(basePlacement(ANCHOR, undefined)).toBeNull()
  })

  it('is null when the base is unreachable or stale', () => {
    const down: BaseGpsPayload = { reachable: false, error: 'timeout' }
    expect(basePlacement(ANCHOR, down)).toBeNull()
    expect(basePlacement(ANCHOR, { ...BASE, stale: true })).toBeNull()
  })

  it('is null when lat/lon are missing or not finite', () => {
    const noLat: BaseGpsPayload = { ...BASE, latitude: undefined }
    const noLon: BaseGpsPayload = { ...BASE, longitude: undefined }
    expect(basePlacement(ANCHOR, noLat)).toBeNull()
    expect(basePlacement(ANCHOR, noLon)).toBeNull()
    expect(basePlacement(ANCHOR, { ...BASE, latitude: Number.NaN })).toBeNull()
    expect(basePlacement(ANCHOR, { ...BASE, longitude: Infinity })).toBeNull()
    expect(basePlacement(ANCHOR, { ...BASE, latitude: '52' as unknown as number })).toBeNull()
    expect(basePlacement(ANCHOR, { ...BASE, latitude: 95 })).toBeNull()
  })
})

describe('baseMarkerLabel', () => {
  it('shows the fix and satellites', () => {
    expect(baseMarkerLabel(BASE)).toBe('Base - RTK Fixed - 18 sats')
  })
  it('omits unknown satellites', () => {
    expect(baseMarkerLabel({ ...BASE, num_satellites: null })).toBe('Base - RTK Fixed')
  })
})
