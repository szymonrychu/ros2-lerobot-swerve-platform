/** Pure planar geometry and hit testing for POIs (map frame, metres). */
import type { Vec2 } from '../map/mapMath'
import { isPointLike, type Poi } from './types'

const DEGENERATE_AREA = 1e-12
/** An object has no radius of its own; only the pointer tolerance counts. */
const OBJECT_HIT_RADIUS_M = 0

/** What a pointer press landed on: a whole POI, or one vertex of an area. */
export interface Hit {
  id: string
  part: 'body' | 'vertex'
  index?: number
}

/**
 * Euclidean distance.
 *
 * @param a - first point
 * @param b - second point
 * @returns distance in the unit of the inputs
 */
export function distance(a: Vec2, b: Vec2): number {
  return Math.hypot(a.x - b.x, a.y - b.y)
}

/**
 * Unsigned polygon area (shoelace).
 *
 * @param polygon - vertices
 * @returns area; 0 for fewer than 3 vertices
 */
export function polygonArea(polygon: [number, number][]): number {
  return Math.abs(signedTwiceArea(polygon)) / 2
}

function signedTwiceArea(polygon: [number, number][]): number {
  let sum = 0
  for (let i = 0; i < polygon.length; i++) {
    const [x0, y0] = polygon[i]
    const [x1, y1] = polygon[(i + 1) % polygon.length]
    sum += x0 * y1 - x1 * y0
  }
  return sum
}

/**
 * Area centroid of a polygon (same formula as poi_store); the vertex mean when the polygon has no area.
 *
 * @param polygon - at least one vertex
 * @returns centroid
 */
export function polygonCentroid(polygon: [number, number][]): Vec2 {
  const twice = signedTwiceArea(polygon)
  if (Math.abs(twice) < DEGENERATE_AREA) {
    const n = polygon.length
    return { x: polygon.reduce((s, p) => s + p[0], 0) / n, y: polygon.reduce((s, p) => s + p[1], 0) / n }
  }
  let cx = 0
  let cy = 0
  for (let i = 0; i < polygon.length; i++) {
    const [x0, y0] = polygon[i]
    const [x1, y1] = polygon[(i + 1) % polygon.length]
    const cross = x0 * y1 - x1 * y0
    cx += (x0 + x1) * cross
    cy += (y0 + y1) * cross
  }
  return { x: cx / (3 * twice), y: cy / (3 * twice) }
}

/**
 * Even-odd point-in-polygon test.
 *
 * @param p - point
 * @param polygon - vertices
 * @returns true when p is inside; false for fewer than 3 vertices
 */
export function pointInPolygon(p: Vec2, polygon: [number, number][]): boolean {
  if (polygon.length < 3) return false
  let inside = false
  for (let i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
    const [xi, yi] = polygon[i]
    const [xj, yj] = polygon[j]
    if (yi > p.y !== yj > p.y && p.x < ((xj - xi) * (p.y - yi)) / (yj - yi) + xi) inside = !inside
  }
  return inside
}

/**
 * Nearest polygon vertex within a tolerance.
 *
 * @param polygon - vertices
 * @param p - query point
 * @param tolerance - maximum distance
 * @returns vertex index, or -1 when none is within tolerance
 */
export function nearestVertex(polygon: [number, number][], p: Vec2, tolerance: number): number {
  let best = -1
  let bestDist = tolerance
  polygon.forEach(([x, y], i) => {
    const d = distance(p, { x, y })
    if (d <= bestDist) {
      best = i
      bestDist = d
    }
  })
  return best
}

/**
 * What is under a map point. Priority: a vertex of the selected area, then the nearest point or object POI (hit radius is
 * the larger of its radius and the tolerance; objects use the tolerance only), then the smallest area containing the point.
 *
 * @param pois - POIs to test
 * @param p - map-frame point
 * @param tolerance - hit tolerance in metres (derive from a pixel size at the current zoom)
 * @param selectedId - id of the selected POI, whose area vertices are grabbable, or null
 * @returns the hit, or null
 */
export function hitTest(pois: Poi[], p: Vec2, tolerance: number, selectedId: string | null): Hit | null {
  const selected = pois.find((q) => q.id === selectedId && q.kind === 'area')
  if (selected) {
    const index = nearestVertex(selected.polygon, p, tolerance)
    if (index >= 0) return { id: selected.id, part: 'vertex', index }
  }
  let nearest: Poi | null = null
  let nearestDist = Infinity
  for (const q of pois) {
    if (!isPointLike(q.kind)) continue
    const d = distance(p, q)
    const radius = q.kind === 'object' ? OBJECT_HIT_RADIUS_M : q.radius_m
    if (d <= Math.max(radius, tolerance) && d < nearestDist) {
      nearest = q
      nearestDist = d
    }
  }
  if (nearest) return { id: nearest.id, part: 'body' }
  let smallest: Poi | null = null
  let smallestArea = Infinity
  for (const q of pois) {
    if (q.kind !== 'area' || !pointInPolygon(p, q.polygon)) continue
    const area = polygonArea(q.polygon)
    if (area < smallestArea) {
      smallest = q
      smallestArea = area
    }
  }
  return smallest ? { id: smallest.id, part: 'body' } : null
}
