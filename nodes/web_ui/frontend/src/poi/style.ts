/** POI presentation (colours by status, creator marker) and list helpers. */
import { CREATOR_OUTLINE, KIND_COLORS, OBJECT_COLOR, SELECTED_OUTLINE, STATUS_COLORS, ZONE_COLORS } from '../map3d/palette'
import type { Poi, PoiKind, PoiList, PoiStatus } from './types'
import { parsePoiTags, type PoiCategory } from './categories'
import { POI_STATUSES, POI_ZONES, ZONE_PCT_MAX, ZONE_PCT_MIN, type PoiZone } from './types'

const DEFAULT_CONFIDENCE = 1

/** Marker shapes: the one cue (besides zone colour) that tells POI kinds apart now that their hues are greyscale. */
export type MarkerShape = 'circle' | 'square' | 'diamond' | 'triangle' | 'hexagon'

/** Shape of each POI kind on the map and in the legend; all distinct. */
export const KIND_SHAPE: Record<PoiKind, MarkerShape> = { point: 'circle', area: 'square', object: 'diamond', photo: 'triangle', marker: 'hexagon' }

const CIRCLE_SEGMENTS = 40
const SHAPE_SIDES: Record<MarkerShape, number> = { circle: CIRCLE_SEGMENTS, square: 4, diamond: 4, triangle: 3, hexagon: 6 }

/**
 * Polygon sides of the filled marker mesh of a kind (a circle is a fine polygon).
 *
 * @param kind - the POI kind
 * @returns the number of polygon sides of KIND_SHAPE[kind]
 */
export function markerSides(kind: PoiKind): number {
  return SHAPE_SIDES[KIND_SHAPE[kind]]
}

/** CSS clip-path of each marker shape, for list and legend swatches. */
export const SHAPE_CLIP: Record<MarkerShape, string> = {
  circle: 'circle(50%)',
  square: 'none',
  diamond: 'polygon(50% 0, 100% 50%, 50% 100%, 0 50%)',
  triangle: 'polygon(50% 0, 100% 100%, 0 100%)',
  hexagon: 'polygon(25% 0, 75% 0, 100% 50%, 75% 100%, 25% 100%, 0 50%)',
}

export { CREATOR_OUTLINE, KIND_COLORS, OBJECT_COLOR, STATUS_COLORS, ZONE_COLORS }
const CLOSED_OPACITY = 0.35
const OPEN_OPACITY = 0.6
const MIN_AREA_VERTICES = 3
const ZONE_OPACITY = { keepout: 0.45, slow: 0.35 } as const
const INACTIVE_ZONE_OPACITY = 0.2
const KEEPOUT_LINE_WIDTH = 4
const SLOW_LINE_WIDTH = 3

/** Colour of each POI category: hue only for the two zone categories, greyscale otherwise (problem spots are the brightest). */
export const CATEGORY_COLORS: Record<PoiCategory, string> = {
  target: KIND_COLORS.point,
  slow: ZONE_COLORS.slow,
  keepout: ZONE_COLORS.keepout,
  problem: STATUS_COLORS.open,
  detail: KIND_COLORS['area-none'],
}
/** Size factor of a survey spot marker (small) and its opacity factor (muted). */
export const SURVEY_SCALE = 0.5
const SURVEY_OPACITY_FACTOR = 0.5
const PROBLEM_LINE_WIDTH = 4
const PROBLEM_OPACITY = 0.8

export interface PoiStyle {
  /** Marker size factor, 1 unless the POI is a survey spot. */
  scale: number
  fill: string
  outline: string
  opacity: number
  lineWidth: number
  dashed: boolean // agent-created POIs have a dashed outline
  creatorTag: 'agent' | 'user'
}

/**
 * Visual style of a POI: fill colour by status (objects: OBJECT_COLOR), outline by creator (agent: blue dashed, user: white solid).
 *
 * @param poi - POI
 * @param selected - whether it is the selected POI
 * @returns style
 */
export function poiStyle(poi: Poi, selected: boolean): PoiStyle {
  const style: PoiStyle = {
    scale: 1,
    fill: poi.kind === 'object' ? OBJECT_COLOR : STATUS_COLORS[poi.status],
    outline: selected ? SELECTED_OUTLINE : CREATOR_OUTLINE[poi.created_by],
    opacity: poi.status === 'open' ? OPEN_OPACITY : CLOSED_OPACITY,
    lineWidth: selected ? 4 : 2,
    dashed: poi.created_by === 'agent',
    creatorTag: poi.created_by,
  }
  if (!hasCategoryStyle(poi)) return style
  const tags = parsePoiTags(poi.note)
  if (tags.category === 'problem') {
    style.lineWidth = selected ? PROBLEM_LINE_WIDTH + 1 : PROBLEM_LINE_WIDTH
    style.opacity = Math.max(style.opacity, PROBLEM_OPACITY)
    style.dashed = false
  }
  if (tags.surveySpot) {
    style.scale = SURVEY_SCALE
    style.opacity *= SURVEY_OPACITY_FACTOR
  }
  return style
}

/**
 * Whether category styling applies: points and areas without a zone (zoned areas keep their zone style, objects and photos theirs).
 *
 * @param poi - POI
 * @returns true for a point or an area with no zone
 */
function hasCategoryStyle(poi: Poi): boolean {
  return (poi.kind === 'point' || (poi.kind === 'area' && (poi.zone ?? 'none') === 'none'))
}

/**
 * Colour of a zone: its zone colour while the POI is open, grey otherwise.
 *
 * @param zone - keepout or slow
 * @param status - POI status
 * @returns CSS colour
 */
export function zoneColor(zone: 'keepout' | 'slow', status: PoiStatus | undefined): string {
  return status === 'open' ? ZONE_COLORS[zone] : STATUS_COLORS.cancelled
}

/**
 * Fill colour of a POI or photo marker: the kind colour, or for an area the zone colour (neutral without a zone).
 * The one rule behind map markers and list swatches.
 *
 * @param kind - POI kind
 * @param zone - area zone
 * @param status - POI status (photos have none)
 * @returns CSS colour
 */
export function fillColor(kind: PoiKind, zone: PoiZone | undefined, status: PoiStatus | undefined): string {
  if (kind !== 'area') return KIND_COLORS[kind]
  return zone === 'keepout' || zone === 'slow' ? zoneColor(zone, status) : KIND_COLORS['area-none']
}

/**
 * Style of an area with a keepout or slow zone: red / amber fill, solid outline; grey and faint unless the POI is open.
 *
 * @param poi - POI
 * @param selected - whether it is the selected POI
 * @returns style, or null when the POI has no zone
 */
export function zoneStyle(poi: Poi, selected: boolean): PoiStyle | null {
  const zone = poi.zone ?? 'none'
  if (zone === 'none') return null
  const active = poi.status === 'open'
  const color = zoneColor(zone, poi.status)
  return {
    scale: 1,
    fill: color,
    outline: selected ? SELECTED_OUTLINE : color,
    opacity: active ? ZONE_OPACITY[zone] : INACTIVE_ZONE_OPACITY,
    lineWidth: selected ? KEEPOUT_LINE_WIDTH + 1 : zone === 'keepout' ? KEEPOUT_LINE_WIDTH : SLOW_LINE_WIDTH,
    dashed: false,
    creatorTag: poi.created_by,
  }
}

/**
 * Fill and outline colours of a map marker. Points and objects: fill by kind, outline by status. Areas: both from the zone
 * (slow amber, keepout red, no zone neutral).
 *
 * @param poi - POI
 * @returns fill and outline colours
 */
export function markerColors(poi: Poi): { fill: string; outline: string } {
  const category = hasCategoryStyle(poi) ? parsePoiTags(poi.note).category : null
  if (category) {
    const color = CATEGORY_COLORS[category]
    return { fill: color, outline: poi.kind === 'area' ? color : STATUS_COLORS[poi.status] }
  }
  if (poi.kind === 'area') {
    const color = fillColor(poi.kind, poi.zone, poi.status)
    return { fill: color, outline: color }
  }
  return { fill: KIND_COLORS[poi.kind], outline: STATUS_COLORS[poi.status] }
}

/**
 * Label text of a zone.
 *
 * @param poi - POI
 * @returns "KEEPOUT", "SLOW 40%", or '' when the POI has no zone
 */
export function zoneLabel(poi: Poi): string {
  if (poi.zone === 'keepout') return 'KEEPOUT'
  if (poi.zone === 'slow') return `SLOW ${poi.speed_limit_pct}%`
  return ''
}

function isNum(v: unknown): v is number {
  return typeof v === 'number' && Number.isFinite(v)
}

function isPoi(raw: unknown): raw is Poi {
  if (!raw || typeof raw !== 'object') return false
  const p = raw as Record<string, unknown>
  if (typeof p.id !== 'string' || !p.id) return false
  if (p.kind !== 'point' && p.kind !== 'area' && p.kind !== 'object' && p.kind !== 'marker') return false
  if (!isNum(p.x) || !isNum(p.y) || !isNum(p.radius_m) || !isNum(p.created_at) || !isNum(p.updated_at)) return false
  if (typeof p.name !== 'string' || typeof p.note !== 'string') return false
  if (!POI_STATUSES.includes(p.status as never) || (p.created_by !== 'agent' && p.created_by !== 'user')) return false
  if (!Array.isArray(p.polygon) || !p.polygon.every((v) => Array.isArray(v) && v.length === 2 && v.every(isNum))) return false
  if (p.kind === 'area' && p.polygon.length < MIN_AREA_VERTICES) return false
  const zone = p.zone ?? 'none'
  if (!POI_ZONES.includes(zone as PoiZone)) return false
  const pct = p.speed_limit_pct
  return zone !== 'slow' || (typeof pct === 'number' && Number.isInteger(pct) && pct >= ZONE_PCT_MIN && pct <= ZONE_PCT_MAX)
}

function withObjectDefaults(p: Poi): Poi {
  return {
    ...p,
    zone: p.zone ?? 'none',
    speed_limit_pct: p.zone === 'slow' ? p.speed_limit_pct : null,
    times_seen: isNum(p.times_seen) ? p.times_seen : 0,
    first_seen: isNum(p.first_seen) ? p.first_seen : 0,
    last_seen: isNum(p.last_seen) ? p.last_seen : 0,
    confidence: isNum(p.confidence) ? p.confidence : DEFAULT_CONFIDENCE,
  }
}

/**
 * Validate a /poi/list payload; malformed entries are dropped.
 *
 * @param raw - topic data
 * @returns the list, or null when raw is not a POI list
 */
export function parsePoiList(raw: unknown): PoiList | null {
  if (!raw || typeof raw !== 'object') return null
  const r = raw as Record<string, unknown>
  if (!Array.isArray(r.pois) || !isNum(r.revision)) return null
  return { pois: r.pois.filter(isPoi).map(withObjectDefaults), revision: r.revision }
}

/**
 * POIs ordered by last update, newest first.
 *
 * @param pois - POIs
 * @returns a sorted copy
 */
export function sortPois(pois: Poi[]): Poi[] {
  return [...pois].sort((a, b) => b.updated_at - a.updated_at)
}

/**
 * Relative age of an update time.
 *
 * @param updatedAt - unix seconds
 * @param now - unix seconds
 * @returns e.g. "30 s ago", "10 min ago", "2 h ago", "3 d ago", or "just now"
 */
export function formatUpdated(updatedAt: number, now: number): string {
  const diff = now - updatedAt
  if (diff < 5) return 'just now'
  if (diff < 60) return `${Math.floor(diff)} s ago`
  if (diff < 3600) return `${Math.floor(diff / 60)} min ago`
  if (diff < 86400) return `${Math.floor(diff / 3600)} h ago`
  return `${Math.floor(diff / 86400)} d ago`
}

/**
 * Sighting summary of an object POI.
 *
 * @param poi - POI
 * @param now - unix seconds
 * @returns e.g. "seen 3x, last seen 1 min ago, confidence 80%"; empty for a POI without sightings
 */
export function objectDetails(poi: Poi, now: number): string {
  const seen = poi.times_seen ?? 0
  if ((poi.kind !== 'object' && poi.kind !== 'marker') || seen < 1) return ''
  const confidence = Math.round((poi.confidence ?? DEFAULT_CONFIDENCE) * 100)
  return `seen ${seen}x, last seen ${formatUpdated(poi.last_seen ?? poi.updated_at, now)}, confidence ${confidence}%`
}

/** Largest tilt of a marker's +z from the horizontal plane for which it still faces a direction, degrees (45 deg from vertical). */
const FACING_MAX_TILT_DEG = 45

/**
 * Map yaw a marker faces: its +z axis (out of the printed face) projected on the floor.
 * Same rule as mcp_server: null when +z is within 45 deg of vertical (a floor or ceiling marker).
 *
 * @param poi - POI
 * @returns yaw in degrees (atan2(y, x), 0 = +x, 90 = +y), or null for a non-marker, a missing orientation or a flat marker
 */
export function facingYaw(poi: Poi): number | null {
  if (poi.kind !== 'marker' || !poi.orientation) return null
  const [x, y, z, w] = poi.orientation
  const nx = 2 * (x * z + w * y)
  const ny = 2 * (y * z - w * x)
  const nz = 1 - 2 * (x * x + y * y)
  const tilt = (Math.atan2(Math.hypot(nx, ny), Math.abs(nz)) * 180) / Math.PI
  if (tilt < FACING_MAX_TILT_DEG) return null
  return (Math.atan2(ny, nx) * 180) / Math.PI
}
