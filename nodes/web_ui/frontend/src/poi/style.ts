/** POI presentation (colours by status, creator marker) and list helpers. */
import type { Poi, PoiList } from './types'
import { POI_STATUSES } from './types'

const DEFAULT_CONFIDENCE = 1

export const STATUS_COLORS = { open: '#ffb000', done: '#2ecc40', cancelled: '#8b949e' } as const
/** Fill of remembered objects (not a status colour, so they stand out from the other POIs). */
export const OBJECT_COLOR = '#b967ff'
export const CREATOR_OUTLINE = { agent: '#2f9bff', user: '#ffffff' } as const
const SELECTED_OUTLINE = '#ff4fd8'
const CLOSED_OPACITY = 0.35
const OPEN_OPACITY = 0.6
const MIN_AREA_VERTICES = 3

export interface PoiStyle {
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
  return {
    fill: poi.kind === 'object' ? OBJECT_COLOR : STATUS_COLORS[poi.status],
    outline: selected ? SELECTED_OUTLINE : CREATOR_OUTLINE[poi.created_by],
    opacity: poi.status === 'open' ? OPEN_OPACITY : CLOSED_OPACITY,
    lineWidth: selected ? 4 : 2,
    dashed: poi.created_by === 'agent',
    creatorTag: poi.created_by,
  }
}

function isNum(v: unknown): v is number {
  return typeof v === 'number' && Number.isFinite(v)
}

function isPoi(raw: unknown): raw is Poi {
  if (!raw || typeof raw !== 'object') return false
  const p = raw as Record<string, unknown>
  if (typeof p.id !== 'string' || !p.id) return false
  if (p.kind !== 'point' && p.kind !== 'area' && p.kind !== 'object') return false
  if (!isNum(p.x) || !isNum(p.y) || !isNum(p.radius_m) || !isNum(p.created_at) || !isNum(p.updated_at)) return false
  if (typeof p.name !== 'string' || typeof p.note !== 'string') return false
  if (!POI_STATUSES.includes(p.status as never) || (p.created_by !== 'agent' && p.created_by !== 'user')) return false
  if (!Array.isArray(p.polygon) || !p.polygon.every((v) => Array.isArray(v) && v.length === 2 && v.every(isNum))) return false
  return p.kind !== 'area' || p.polygon.length >= MIN_AREA_VERTICES
}

function withObjectDefaults(p: Poi): Poi {
  return {
    ...p,
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
  if (poi.kind !== 'object' || seen < 1) return ''
  const confidence = Math.round((poi.confidence ?? DEFAULT_CONFIDENCE) * 100)
  return `seen ${seen}x, last seen ${formatUpdated(poi.last_seen ?? poi.updated_at, now)}, confidence ${confidence}%`
}
