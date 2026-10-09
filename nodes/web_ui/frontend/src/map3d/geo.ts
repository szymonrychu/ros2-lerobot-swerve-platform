/**
 * GPS map layer math: Web Mercator (slippy map) tiles and placement of lat/lon in the ROS map frame.
 *
 * The backend publishes '/web_ui/gps_anchor' as {lat, lon, heading_rad, residual_m, n_points, source} (or null while no anchor
 * is available): (lat, lon) is the GPS position of the map origin (0, 0) and heading_rad is the angle of the map +x
 * axis measured counter-clockwise from East, i.e. enu = R(heading_rad) * map. Around the anchor a local tangent plane
 * (equirectangular) approximation is used, which is accurate to centimetres over the few hundred metres the layer
 * covers.
 */
import { Vec2 } from '../map/mapMath'

export const EARTH_RADIUS_M = 6378137
export const EARTH_CIRCUMFERENCE_M = 2 * Math.PI * EARTH_RADIUS_M // 40075016.686 m
export const MAX_MERCATOR_LAT = 85.0511287798

/** GPS placement of the map frame (payload of /web_ui/gps_anchor). */
export interface GpsAnchor {
  lat: number // degrees, latitude of map (0, 0)
  lon: number // degrees, longitude of map (0, 0)
  heading_rad: number // map +x axis, counter-clockwise from East
  residual_m?: number // fit residual (diagnostic)
  n_points?: number // samples used by the fit (diagnostic)
  source?: AnchorSource // how the anchor was obtained
}

/** 'fit' = drive-based rigid fit, 'compass' = one fix plus the IMU compass heading (while parked). */
export type AnchorSource = 'fit' | 'compass'

export interface LatLon {
  latitude: number
  longitude: number
}

/** One tile placed on the map ground. */
export interface PlacedTile {
  z: number
  x: number
  y: number
  dx: number // column offset from the centre tile
  dy: number // row offset from the centre tile (down = south)
  url: string
  center: Vec2 // map frame, metres
  width: number // metres along the tile's east edge direction
  height: number // metres along the tile's north edge direction
  yaw: number // rotation of the tile's east axis relative to map +x (radians)
}

const DEG = Math.PI / 180

/**
 * Validate an anchor payload.
 *
 * @param raw - topic payload (any shape; null when the backend has no fit yet)
 * @returns a GpsAnchor, or null when lat, lon or heading_rad is missing, not finite or out of range; residual_m and
 *   n_points are kept only when they are finite numbers
 */
export function validAnchor(raw: unknown): GpsAnchor | null {
  if (!raw || typeof raw !== 'object') return null
  const a = raw as Record<string, unknown>
  const num = (v: unknown): v is number => typeof v === 'number' && Number.isFinite(v)
  if (!num(a.lat) || !num(a.lon) || !num(a.heading_rad)) return null
  if (Math.abs(a.lat) > 90 || Math.abs(a.lon) > 180) return null
  const out: GpsAnchor = { lat: a.lat, lon: a.lon, heading_rad: a.heading_rad }
  if (num(a.residual_m)) out.residual_m = a.residual_m
  if (num(a.n_points)) out.n_points = a.n_points
  if (a.source === 'fit' || a.source === 'compass') out.source = a.source
  return out
}

/**
 * One-line description of how the anchor was obtained.
 *
 * @param anchor - validated anchor
 * @returns 'compass heading', or 'drive fit' with the sample count and residual when the payload has them
 */
export function anchorSummary(anchor: GpsAnchor): string {
  if (anchor.source === 'compass') return 'compass heading'
  const parts = ['drive fit']
  if (anchor.n_points !== undefined) parts.push(`${anchor.n_points} pts`)
  if (anchor.residual_m !== undefined) parts.push(`residual ${anchor.residual_m.toFixed(2)} m`)
  return parts.join(', ')
}

/**
 * Fractional slippy-map tile coordinates of a lat/lon.
 *
 * @param latitude - degrees
 * @param longitude - degrees
 * @param zoom - tile zoom level
 * @returns {x, y} fractional tile coordinates (floor gives the tile index)
 */
export function latLonToTile(latitude: number, longitude: number, zoom: number): Vec2 {
  const n = 2 ** zoom
  const lat = Math.max(-MAX_MERCATOR_LAT, Math.min(MAX_MERCATOR_LAT, latitude)) * DEG
  return {
    x: ((longitude + 180) / 360) * n,
    y: ((1 - Math.asinh(Math.tan(lat)) / Math.PI) / 2) * n,
  }
}

/**
 * Lat/lon of fractional tile coordinates (integer values give a tile's north-west corner).
 *
 * @param x - fractional tile x
 * @param y - fractional tile y
 * @param zoom - tile zoom level
 * @returns latitude/longitude in degrees
 */
export function tileToLatLon(x: number, y: number, zoom: number): LatLon {
  const n = 2 ** zoom
  return {
    latitude: Math.atan(Math.sinh(Math.PI * (1 - (2 * y) / n))) / DEG,
    longitude: (x / n) * 360 - 180,
  }
}

/**
 * Ground width of one tile at a latitude.
 *
 * @param latitude - degrees
 * @param zoom - tile zoom level
 * @returns metres
 */
export function tileSizeMeters(latitude: number, zoom: number): number {
  return (EARTH_CIRCUMFERENCE_M * Math.cos(latitude * DEG)) / 2 ** zoom
}

/**
 * URL of a tile on the backend tile proxy.
 *
 * @param z - zoom
 * @param x - tile x
 * @param y - tile y
 * @returns '/api/tiles/{z}/{x}/{y}.png'
 */
export function tileUrl(z: number, x: number, y: number): string {
  return `/api/tiles/${z}/${x}/${y}.png`
}

/**
 * Local east/north offset of a lat/lon from a reference (equirectangular approximation).
 *
 * @returns {x: east, y: north} in metres
 */
export function latLonToEnu(latitude: number, longitude: number, refLat: number, refLon: number): Vec2 {
  return {
    x: (longitude - refLon) * DEG * EARTH_RADIUS_M * Math.cos(refLat * DEG),
    y: (latitude - refLat) * DEG * EARTH_RADIUS_M,
  }
}

/**
 * Inverse of latLonToEnu.
 *
 * @param east - metres east of the reference
 * @param north - metres north of the reference
 * @returns latitude/longitude in degrees
 */
export function enuToLatLon(east: number, north: number, refLat: number, refLon: number): LatLon {
  return {
    latitude: refLat + north / EARTH_RADIUS_M / DEG,
    longitude: refLon + east / (EARTH_RADIUS_M * Math.cos(refLat * DEG)) / DEG,
  }
}

/**
 * Map-frame point of a lat/lon using the anchor.
 *
 * @param anchor - GPS anchor
 * @param latitude - degrees
 * @param longitude - degrees
 * @returns map point {x, y} in metres
 */
export function latLonToMap(anchor: GpsAnchor, latitude: number, longitude: number): Vec2 {
  const enu = latLonToEnu(latitude, longitude, anchor.lat, anchor.lon)
  // ENU = R(heading) * map  =>  map = R(-heading) * ENU
  const c = Math.cos(anchor.heading_rad)
  const s = Math.sin(anchor.heading_rad)
  return { x: c * enu.x + s * enu.y, y: -s * enu.x + c * enu.y }
}

/**
 * Lat/lon of a map-frame point using the anchor.
 *
 * @param anchor - GPS anchor
 * @param x - map x in metres
 * @param y - map y in metres
 * @returns latitude/longitude in degrees
 */
export function mapToLatLon(anchor: GpsAnchor, x: number, y: number): LatLon {
  const c = Math.cos(anchor.heading_rad)
  const s = Math.sin(anchor.heading_rad)
  return enuToLatLon(c * x - s * y, s * x + c * y, anchor.lat, anchor.lon)
}

/**
 * Tiles covering a square around a map point, each placed in the map frame.
 *
 * @param anchor - GPS anchor
 * @param around - map point the grid is centred on (usually the robot)
 * @param zoom - tile zoom level
 * @param radius - tiles on each side of the centre tile ((2r+1)^2 tiles)
 * @returns placed tiles (indices clamped/wrapped to the world)
 */
export function tilesAround(anchor: GpsAnchor, around: Vec2, zoom: number, radius: number): PlacedTile[] {
  const n = 2 ** zoom
  const ll = mapToLatLon(anchor, around.x, around.y)
  const centre = latLonToTile(ll.latitude, ll.longitude, zoom)
  const cx = Math.floor(centre.x)
  const cy = Math.floor(centre.y)
  const tiles: PlacedTile[] = []
  for (let dy = -radius; dy <= radius; dy++) {
    const ty = cy + dy
    if (ty < 0 || ty >= n) continue
    for (let dx = -radius; dx <= radius; dx++) {
      const rawX = cx + dx
      const tx = ((rawX % n) + n) % n
      // Geometry uses the unwrapped column so tiles stay adjacent across the antimeridian.
      const nw = latLonToMap(anchor, ...latLonPair(tileToLatLon(rawX, ty, zoom)))
      const se = latLonToMap(anchor, ...latLonPair(tileToLatLon(rawX + 1, ty + 1, zoom)))
      const ne = latLonToMap(anchor, ...latLonPair(tileToLatLon(rawX + 1, ty, zoom)))
      const mid = latLonToMap(anchor, ...latLonPair(tileToLatLon(rawX + 0.5, ty + 0.5, zoom)))
      tiles.push({
        z: zoom,
        x: tx,
        y: ty,
        dx,
        dy,
        url: tileUrl(zoom, tx, ty),
        center: mid,
        width: Math.hypot(ne.x - nw.x, ne.y - nw.y),
        height: Math.hypot(se.x - ne.x, se.y - ne.y),
        yaw: -anchor.heading_rad,
      })
    }
  }
  return tiles
}

function latLonPair(ll: LatLon): [number, number] {
  return [ll.latitude, ll.longitude]
}
