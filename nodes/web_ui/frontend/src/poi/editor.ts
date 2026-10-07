/** Pure editor state logic for POIs: area drafting, new-POI fields, drag previews and update commands. */
import type { Vec2 } from '../map/mapMath'
import { polygonCentroid, distance, type Hit } from './geometry'
import type { Poi, PoiCommand, PoiFields, PoiKind } from './types'

/** A press that moves less than this (screen px) is a click. */
export const CLICK_MAX_DRAG_PX = 6
/** An added vertex closer than this to the previous one is dropped (the second click of a double click). */
export const MIN_VERTEX_SPACING_M = 0.03
export const MIN_AREA_VERTICES = 3
export const DEFAULT_RADIUS_M = 0.2
/** Hit tolerance in screen px, converted to metres at the current zoom. */
export const HIT_TOLERANCE_PX = 14

export type PoiMode = 'none' | 'add_point' | 'add_area'

/** An in-progress drag of a POI (body or area vertex). */
export interface DragState {
  target: Hit
  startMap: Vec2
  startScreen: Vec2
  current: Vec2
  movedPx: number
}

/**
 * Whether a press-release pair is a click rather than a drag.
 *
 * @param down - press position, CSS px
 * @param up - release position, CSS px
 * @returns true when the pointer moved less than CLICK_MAX_DRAG_PX
 */
export function isClick(down: Vec2, up: Vec2): boolean {
  return distance(down, up) < CLICK_MAX_DRAG_PX
}

/**
 * Append a vertex to an area draft.
 *
 * @param draft - vertices so far
 * @param p - new vertex
 * @returns a new draft; unchanged copy when p coincides with the last vertex
 */
export function addVertex(draft: Vec2[], p: Vec2): Vec2[] {
  const last = draft[draft.length - 1]
  if (last && distance(last, p) < MIN_VERTEX_SPACING_M) return draft
  return [...draft, p]
}

/**
 * Whether an area draft has enough vertices.
 *
 * @param draft - vertices
 * @returns true for at least MIN_AREA_VERTICES
 */
export function canFinishArea(draft: Vec2[]): boolean {
  return draft.length >= MIN_AREA_VERTICES
}

/**
 * Fields of a new point POI.
 *
 * @param at - map-frame position
 * @param name - label
 * @returns fields to send with op "add"
 */
export function newPointFields(at: Vec2, name: string): PoiFields {
  return {
    kind: 'point',
    x: at.x,
    y: at.y,
    polygon: [],
    radius_m: DEFAULT_RADIUS_M,
    name,
    note: '',
    status: 'open',
    created_by: 'user',
  }
}

/**
 * Fields of a new area POI.
 *
 * @param vertices - at least three map-frame vertices
 * @param name - label
 * @returns fields to send with op "add" (x, y is the centroid; the store recomputes it)
 */
export function newAreaFields(vertices: Vec2[], name: string): PoiFields {
  const polygon = vertices.map((v): [number, number] => [v.x, v.y])
  const c = polygonCentroid(polygon)
  return {
    kind: 'area',
    x: c.x,
    y: c.y,
    polygon,
    radius_m: DEFAULT_RADIUS_M,
    name,
    note: '',
    status: 'open',
    created_by: 'user',
  }
}

/**
 * Default label for a new POI: "Point n" / "Area n" with n one above the existing count of that kind.
 *
 * @param kind - POI kind
 * @param pois - existing POIs
 * @returns label
 */
export function defaultName(kind: PoiKind, pois: Poi[]): string {
  const n = pois.filter((p) => p.kind === kind).length + 1
  return `${kind === 'point' ? 'Point' : 'Area'} ${n}`
}

/**
 * Begin dragging.
 *
 * @param target - what was pressed
 * @param startMap - map point under the press
 * @param startScreen - press position, CSS px
 * @returns new drag
 */
export function startDrag(target: Hit, startMap: Vec2, startScreen: Vec2): DragState {
  return { target, startMap, startScreen, current: startMap, movedPx: 0 }
}

/**
 * Follow the pointer.
 *
 * @param drag - current drag
 * @param currentMap - map point under the pointer, or null when it misses the ground
 * @param currentScreen - pointer position, CSS px
 * @returns updated drag
 */
export function updateDrag(drag: DragState, currentMap: Vec2 | null, currentScreen: Vec2): DragState {
  return {
    ...drag,
    current: currentMap ?? drag.current,
    movedPx: Math.max(drag.movedPx, distance(currentScreen, drag.startScreen)),
  }
}

/**
 * The POI as it looks while dragged.
 *
 * @param poi - POI being dragged
 * @param drag - current drag
 * @returns moved copy (a point moves, an area translates or has one vertex moved with its centroid recomputed)
 */
export function dragPreview(poi: Poi, drag: DragState): Poi {
  const dx = drag.current.x - drag.startMap.x
  const dy = drag.current.y - drag.startMap.y
  if (poi.kind === 'point') return { ...poi, x: poi.x + dx, y: poi.y + dy }
  const polygon = poi.polygon.map(([x, y], i): [number, number] =>
    drag.target.part === 'vertex' && drag.target.index !== i ? [x, y] : [x + dx, y + dy],
  )
  const c = polygonCentroid(polygon)
  return { ...poi, polygon, x: c.x, y: c.y }
}

/**
 * The update command that commits a drag.
 *
 * @param poi - POI being dragged (before the drag)
 * @param drag - finished drag
 * @returns op "update" with id and the moved geometry
 */
export function dragCommand(poi: Poi, drag: DragState): Extract<PoiCommand, { op: 'update' }> {
  const moved = dragPreview(poi, drag)
  return {
    op: 'update',
    poi: poi.kind === 'point' ? { id: poi.id, x: moved.x, y: moved.y } : { id: poi.id, polygon: moved.polygon },
  }
}

/**
 * Replace the POI that has a local (optimistic) override.
 *
 * @param pois - POIs from the store
 * @param override - locally moved POI, or null
 * @returns pois with the matching entry replaced
 */
export function applyOverride(pois: Poi[], override: Poi | null): Poi[] {
  if (!override) return pois
  return pois.map((p) => (p.id === override.id ? override : p))
}
