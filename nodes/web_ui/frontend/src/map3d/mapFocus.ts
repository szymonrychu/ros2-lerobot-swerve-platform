/** Resolve a POI-tab "Show on map" request to a map point plus the thing to select (the map's popup). */
import type { Photo } from '../photos/api'
import { polygonCentroid } from '../poi/geometry'
import type { PendingFocus } from '../poi/poiItems'
import type { Poi } from '../poi/types'
import type { Vec2 } from '../map/mapMath'

export interface ResolvedFocus {
  /** Map-frame point to centre on. */
  point: Vec2
  /** POI to select, or null for a photo. */
  poiId: string | null
  /** Photo to open, or null for a POI. */
  photo: Photo | null
}

/**
 * @param focus - the pending request
 * @param pois - POIs currently known to the map
 * @param photos - photos currently loaded by the map's photo layer
 * @returns where to centre and what to open, or null while the target is unknown or unlocated
 */
export function resolveFocus(focus: PendingFocus, pois: Poi[], photos: Photo[]): ResolvedFocus | null {
  if (focus.kind === 'poi') {
    const p = pois.find((q) => q.id === focus.id)
    if (!p) return null
    const at = p.kind === 'area' && p.polygon.length > 0 ? polygonCentroid(p.polygon) : p
    return { point: { x: at.x, y: at.y }, poiId: p.id, photo: null }
  }
  const ph = photos.find((q) => q.id === focus.id)
  if (!ph || ph.x === null || ph.y === null) return null
  return { point: { x: ph.x, y: ph.y }, poiId: null, photo: ph }
}
