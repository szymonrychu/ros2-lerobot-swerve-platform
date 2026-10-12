/**
 * "Show on map" from the POI tab: resolve a pending focus to a map point plus the item its popup shows, and wait
 * for the scene controller (the map tab mounts its lazy 3D chunk after the switch) before centring on it.
 */
import type { Photo } from '../photos/api'
import { isLocated, toPoiItems, type PendingFocus, type PoiItem } from '../poi/poiItems'
import type { Poi } from '../poi/types'
import type { Vec2 } from '../map/mapMath'
import type { SceneController } from './MapScene'

/** How often to look for the scene controller while the 3D chunk loads, ms. */
export const CONTROLLER_POLL_MS = 100

/** What the map centres on and what its popup shows. */
export interface FocusTarget {
  /** Map-frame point the popup arrow points at (an area's centroid). */
  anchor: Vec2
  item: PoiItem
}

/**
 * @param request - the pending request
 * @param pois - POIs currently known to the map
 * @param photos - photos currently known to the map
 * @returns the anchor and item, or null for an unknown id or an unlocated photo
 */
export function resolveFocus(request: PendingFocus, pois: Poi[], photos: Photo[]): FocusTarget | null {
  const items =
    request.kind === 'poi'
      ? toPoiItems(pois.filter((p) => p.id === request.id), [])
      : toPoiItems([], photos.filter((p) => p.id === request.id))
  const item = items[0]
  if (!item || !isLocated(item)) return null
  return { anchor: { x: item.x as number, y: item.y as number }, item }
}

/**
 * Run once the scene controller exists: at once when it already does, otherwise on the first poll that finds it.
 *
 * @param getController - reads the current controller (the tab's controllerRef)
 * @param run - called exactly once with the controller
 * @param intervalMs - poll period
 * @returns cancel; stops polling (no-op after run)
 */
export function whenController(
  getController: () => SceneController | null,
  run: (controller: SceneController) => void,
  intervalMs: number = CONTROLLER_POLL_MS,
): () => void {
  const now = getController()
  if (now) {
    run(now)
    return () => undefined
  }
  const timer = setInterval(() => {
    const c = getController()
    if (!c) return
    clearInterval(timer)
    run(c)
  }, intervalMs)
  return () => clearInterval(timer)
}

/**
 * Consume a pending focus: once the target resolves and the controller exists, centre on it once and report it.
 * An unresolved target (unknown id, unlocated photo, data still loading) is left pending: nothing happens.
 *
 * @param request - the pending request
 * @param pois - POIs currently known to the map
 * @param photos - photos currently known to the map
 * @param getController - reads the current controller
 * @param onFocused - called once after centring, with the resolved target (open the popup, clear the request)
 * @param intervalMs - controller poll period
 * @returns cancel; stops waiting for the controller
 */
export function consumeFocus(
  request: PendingFocus,
  pois: Poi[],
  photos: Photo[],
  getController: () => SceneController | null,
  onFocused: (target: FocusTarget) => void,
  intervalMs: number = CONTROLLER_POLL_MS,
): () => void {
  const target = resolveFocus(request, pois, photos)
  if (!target) return () => undefined
  return whenController(
    getController,
    (c) => {
      c.centerOn(target.anchor)
      onFocused(target)
    },
    intervalMs,
  )
}
