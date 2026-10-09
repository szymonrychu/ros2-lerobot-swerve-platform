/** Placement of the RTK base station on the GPS map layer. */
import { Vec2 } from '../map/mapMath'
import { GpsAnchor, latLonToMap } from '../map3d/geo'
import type { BaseGpsPayload, BaseGpsSample } from './gpsStatus'

/**
 * Map-frame position of the RTK base station.
 *
 * @param anchor - validated GPS anchor, or null while there is none
 * @param base - latest polled base status (undefined/null when none received)
 * @returns {x, y} in metres in the map frame, or null when the anchor is missing, the base is unreachable or stale,
 *   or its latitude/longitude is missing, not a finite number or out of range
 */
export function basePlacement(anchor: GpsAnchor | null, base: BaseGpsPayload | null | undefined): Vec2 | null {
  if (!anchor || !base || !base.reachable || base.stale) return null
  const { latitude, longitude } = base
  if (typeof latitude !== 'number' || typeof longitude !== 'number') return null
  if (!Number.isFinite(latitude) || !Number.isFinite(longitude)) return null
  if (Math.abs(latitude) > 90 || Math.abs(longitude) > 180) return null
  return latLonToMap(anchor, latitude, longitude)
}

/**
 * Marker label text.
 *
 * @param base - reachable base status
 * @returns 'Base - <fix>' with ' - <N> sats' when the satellite count is known
 */
export function baseMarkerLabel(base: BaseGpsSample): string {
  const sats = base.num_satellites === null ? null : `${base.num_satellites} sats`
  return ['Base', base.fix, sats].filter(Boolean).join(' - ')
}
