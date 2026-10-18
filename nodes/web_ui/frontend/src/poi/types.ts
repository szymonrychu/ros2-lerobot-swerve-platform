/** POI contract (poi_store, map frame) as seen by the UI; see nodes/poi_store/README.md. */

/**
 * 'object' is a remembered object: point-like (position only), name = label, with sighting fields.
 * 'photo' is a camera photo (never on /poi/list; served by /api/photos), linked to other POIs through poi_ids.
 */
export type PoiKind = 'point' | 'area' | 'object' | 'photo'
export type PoiStatus = 'open' | 'done' | 'cancelled'
export type PoiCreator = 'agent' | 'user'
/** Nav2 zone of an area: keepout (no entry) or slow (speed cap); active only while the POI is open. */
export type PoiZone = 'none' | 'keepout' | 'slow'

export const POI_ZONES: readonly PoiZone[] = ['none', 'keepout', 'slow']
/** Allowed speed_limit_pct range of a slow zone (percent). */
export const ZONE_PCT_MIN = 10
export const ZONE_PCT_MAX = 100

export const POI_STATUSES: readonly PoiStatus[] = ['open', 'done', 'cancelled']
export const POI_NAME_MAX = 60
export const POI_NOTE_MAX = 2000

export interface Poi {
  id: string
  kind: PoiKind
  frame: 'map'
  x: number // point position, or the polygon centroid of an area
  y: number
  polygon: [number, number][] // area vertices (>= 3); [] for a point
  radius_m: number // point radius
  name: string
  note: string
  status: PoiStatus
  created_by: PoiCreator
  created_at: number // unix seconds
  updated_at: number
  times_seen?: number // objects: sighting count (0 for other kinds)
  first_seen?: number // objects: unix seconds of the first sighting
  last_seen?: number // objects: unix seconds of the latest sighting
  confidence?: number // objects: 0..1
  zone?: PoiZone // areas only; absent means 'none'
  speed_limit_pct?: number | null // slow zones: ZONE_PCT_MIN..ZONE_PCT_MAX, else null
  poi_ids?: string[] // photos: ids of the linked non-photo POIs
}

/**
 * Whether a POI kind is a single map position (point or object) rather than a polygon.
 *
 * @param kind - POI kind
 * @returns true for 'point' and 'object'
 */
export function isPointLike(kind: PoiKind): boolean {
  return kind !== 'area'
}

export interface PoiList {
  pois: Poi[]
  revision: number
}

/** Fields of a POI the UI sends on add (the store assigns id and timestamps). */
export type PoiFields = Omit<Poi, 'id' | 'frame' | 'created_at' | 'updated_at' | 'times_seen' | 'first_seen' | 'last_seen' | 'confidence'>

export type PoiCommand =
  | { op: 'add'; poi: Partial<Poi> }
  | { op: 'update'; poi: Partial<Poi> & { id: string } }
  | { op: 'delete'; poi: { id: string } }
  | { op: 'delete_photos'; selector: Record<string, unknown> }
  | { op: 'clear'; created_by?: PoiCreator | null; kinds?: PoiKind[]; statuses?: PoiStatus[]; keep_zones?: boolean; ids?: string[] }

export interface PoiResult {
  ok: boolean
  message: string
  poi: Poi | null
  /** Clear op only: number of POIs the store removed. */
  removed?: number
  /** Clear op only: POIs and photos removed (removed is their sum). */
  pois?: number
  photos?: number
}
