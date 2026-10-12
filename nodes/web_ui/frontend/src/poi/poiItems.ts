/** One list model merging POIs and photos for the POI tab: filter, search, sort, map focus. */
import type { Photo } from '../photos/api'
import { polygonCentroid } from './geometry'
import type { PoiKind, PoiStatus, PoiZone, Poi } from './types'

const KEY_SEPARATOR = ':'

export type PoiItemSource = 'poi' | 'photo'

export interface PoiItem {
  /** Unique across both sources: "<source>:<id>". */
  key: string
  kind: PoiKind
  name: string
  /** Map position; null for an unlocated photo. Areas use their centroid. */
  x: number | null
  y: number | null
  status?: PoiStatus
  zone?: PoiZone
  note: string
  /** Unix seconds: POI updated_at, photo capture time. */
  updated: number
  source: PoiItemSource
}

/** A request for the map to centre on, and open the popup of, one POI or photo. */
export interface PendingFocus {
  kind: PoiItemSource
  id: string
}

/**
 * Merge POIs and photos into one item list.
 *
 * @param pois - POIs (an area item takes its polygon centroid as x/y)
 * @param photos - photos (null x/y stay null)
 * @returns POI items first, then photo items, with keys unique across both
 */
export function toPoiItems(pois: Poi[], photos: Photo[]): PoiItem[] {
  const fromPois = pois.map((p): PoiItem => {
    const at = p.kind === 'area' && p.polygon.length > 0 ? polygonCentroid(p.polygon) : p
    return {
      key: `poi${KEY_SEPARATOR}${p.id}`, kind: p.kind, name: p.name, x: at.x, y: at.y,
      status: p.status, zone: p.zone, note: p.note, updated: p.updated_at, source: 'poi',
    }
  })
  const fromPhotos = photos.map(
    (p): PoiItem => ({
      key: `photo${KEY_SEPARATOR}${p.id}`, kind: 'photo', name: p.name, x: p.x, y: p.y, note: p.note, updated: p.ts, source: 'photo',
    }),
  )
  return [...fromPois, ...fromPhotos]
}

/**
 * Filter items by kind and a search text.
 *
 * @param items - items to filter
 * @param kinds - kinds to keep; an empty set keeps every kind
 * @param query - case-insensitive substring searched in name and note; blank matches all
 * @returns the matching items, in input order
 */
export function filterItems(items: PoiItem[], kinds: Set<PoiKind>, query: string): PoiItem[] {
  const q = query.trim().toLowerCase()
  return items.filter(
    (i) => (kinds.size === 0 || kinds.has(i.kind)) && (q === '' || i.name.toLowerCase().includes(q) || i.note.toLowerCase().includes(q)),
  )
}

/**
 * Order items by update time, newest first (the sortPois order for POIs, capture time for photos).
 *
 * @param items - items to order
 * @returns a sorted copy
 */
export function sortItems(items: PoiItem[]): PoiItem[] {
  return [...items].sort((a, b) => b.updated - a.updated)
}

/**
 * Whether an item has a map position.
 *
 * @param item - item
 * @returns true when both x and y are known
 */
export function isLocated(item: PoiItem): boolean {
  return item.x !== null && item.y !== null
}

/**
 * The POI or photo id of an item (its key without the source prefix).
 *
 * @param item - item
 * @returns the id in its source store
 */
export function itemId(item: PoiItem): string {
  return item.key.slice(item.source.length + KEY_SEPARATOR.length)
}

/**
 * The map focus request for an item.
 *
 * @param item - item
 * @returns the request, or null when the item is unlocated
 */
export function focusRequestFor(item: PoiItem): PendingFocus | null {
  if (!isLocated(item)) return null
  return { kind: item.source, id: itemId(item) }
}
