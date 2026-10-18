/** Pure helpers for filtering, marking and batch-deleting POIs in the list. */

import { parsePoiTags } from './categories'
import type { Poi } from './types'

/** Category of POIs that carry neither a tag nor a Nav2 zone. */
export const UNCATEGORISED = 'uncategorised'

export interface MarkedSummary {
  count: number
  byCategory: Record<string, number>
  keepouts: Poi[]
}

/**
 * Category of a POI for filtering.
 *
 * @param poi - the POI
 * @returns the tag category, else 'keepout' / 'slow' from the zone, else UNCATEGORISED
 */
export function poiCategory(poi: Poi): string {
  const tagged = parsePoiTags(poi.note).category
  if (tagged !== null) return tagged
  if (poi.zone === 'keepout') return 'keepout'
  if (poi.zone === 'slow') return 'slow'
  return UNCATEGORISED
}

/**
 * Keep the POIs in the selected categories.
 *
 * @param pois - all POIs
 * @param categories - selected categories; empty means no filter
 * @returns the POIs passing the filter, in input order
 */
export function filterPois(pois: Poi[], categories: ReadonlySet<string>): Poi[] {
  return categories.size === 0 ? pois : pois.filter((p) => categories.has(poiCategory(p)))
}

/**
 * Flip the mark of one POI.
 *
 * @param marked - marked ids
 * @param id - POI id
 * @returns a new set with the id added or removed
 */
export function toggleMarked(marked: ReadonlySet<string>, id: string): Set<string> {
  const next = new Set(marked)
  if (!next.delete(id)) next.add(id)
  return next
}

/**
 * Mark every POI passing the current filter.
 *
 * @param visible - POIs passing the filter
 * @returns the set of their ids
 */
export function markAllVisible(visible: Poi[]): Set<string> {
  return new Set(visible.map((p) => p.id))
}

/**
 * Drop marks of POIs that are no longer visible or no longer exist.
 *
 * @param marked - marked ids
 * @param visible - POIs passing the filter
 * @returns the marked ids that are still visible
 */
export function pruneMarked(marked: ReadonlySet<string>, visible: Poi[]): Set<string> {
  const ids = new Set(visible.map((p) => p.id))
  return new Set([...marked].filter((id) => ids.has(id)))
}

/**
 * Describe the marked POIs for the delete confirmation.
 *
 * @param pois - all POIs
 * @param marked - marked ids
 * @returns count of existing marked POIs, count per category, and marked keep-out zones (zone decides, not the tag)
 */
export function summariseMarked(pois: Poi[], marked: ReadonlySet<string>): MarkedSummary {
  const hit = pois.filter((p) => marked.has(p.id))
  const byCategory: Record<string, number> = {}
  for (const p of hit) {
    const c = poiCategory(p)
    byCategory[c] = (byCategory[c] ?? 0) + 1
  }
  return { count: hit.length, byCategory, keepouts: hit.filter((p) => p.zone === 'keepout') }
}
