/** POI contract (poi_store, map frame) as seen by the UI; see nodes/poi_store/README.md. */

export type PoiKind = 'point' | 'area'
export type PoiStatus = 'open' | 'done' | 'cancelled'
export type PoiCreator = 'agent' | 'user'

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
}

export interface PoiList {
  pois: Poi[]
  revision: number
}

/** Fields of a POI the UI sends on add (the store assigns id and timestamps). */
export type PoiFields = Omit<Poi, 'id' | 'frame' | 'created_at' | 'updated_at'>

export type PoiCommand =
  | { op: 'add'; poi: Partial<Poi> }
  | { op: 'update'; poi: Partial<Poi> & { id: string } }
  | { op: 'delete'; poi: { id: string } }

export interface PoiResult {
  ok: boolean
  message: string
  poi: Poi | null
}
