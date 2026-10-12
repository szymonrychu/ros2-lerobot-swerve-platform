/** Pure geometry and filters of the map photo layer (map frame, metres and radians). */
import { PHOTO_COLOR } from '../map3d/palette'
import type { Photo } from './api'

/** Length of a viewing-direction wedge, m. */
export const WEDGE_LENGTH_M = 0.175
/** Display-only wedge width when the field of view is unknown (60 deg; not data, the wedge is dashed). */
export const DEFAULT_HFOV_RAD = Math.PI / 3
/** Circumradius of the triangle that marks a photo on the map, m (readable at phone zoom). */
export const PHOTO_MARKER_RADIUS_M = 0.07
/** Most photos the map layer draws. */
export const LAYER_MAX_PHOTOS = 1000
const SECONDS_PER_HOUR = 3600
const SECONDS_PER_DAY = 86400

/** Age filter of the layer. */
export type PhotoAge = '24h' | '7d' | 'all'
export const PHOTO_AGES: readonly PhotoAge[] = ['24h', '7d', 'all']
export const PHOTO_AGE_LABELS: Record<PhotoAge, string> = { '24h': '24 h', '7d': '7 d', all: 'All' }

/** A viewing-direction wedge: apex first, then the left and right edge ends. */
export interface Wedge {
  vertices: [Vec2Tuple, Vec2Tuple, Vec2Tuple]
  /** True when the field of view is unknown and the width is only a display default. */
  dashed: boolean
}
type Vec2Tuple = [number, number]

/**
 * Wedge of a photo in map coordinates.
 *
 * @param photo - the photo
 * @returns apex at the camera position, edges at yaw +- hfov/2 (default width when hfov is null), or null when unlocated
 */
export function photoWedge(photo: Photo): Wedge | null {
  const v = photo.view
  if (v === null) return null
  const hfov = v.hfov ?? DEFAULT_HFOV_RAD
  const edge = (a: number): Vec2Tuple => [v.x + WEDGE_LENGTH_M * Math.cos(a), v.y + WEDGE_LENGTH_M * Math.sin(a)]
  return { vertices: [[v.x, v.y], edge(v.yaw + hfov / 2), edge(v.yaw - hfov / 2)], dashed: v.hfov === null }
}

/**
 * The triangle marker (KIND_SHAPE.photo) of a located photo.
 *
 * @param photo - the photo
 * @returns three map-frame vertices on a circle of PHOTO_MARKER_RADIUS_M around the camera position, the first along the yaw; null when unlocated
 */
export function photoMarker(photo: Photo): [Vec2Tuple, Vec2Tuple, Vec2Tuple] | null {
  const v = photo.view
  if (v === null) return null
  const vertex = (a: number): Vec2Tuple => [v.x + PHOTO_MARKER_RADIUS_M * Math.cos(a), v.y + PHOTO_MARKER_RADIUS_M * Math.sin(a)]
  const third = (2 * Math.PI) / 3
  return [vertex(v.yaw), vertex(v.yaw + third), vertex(v.yaw - third)]
}

/**
 * The newest photos.
 *
 * @param photos - any order
 * @param limit - most to keep
 * @returns a new array, newest first, at most `limit` long
 */
export function newestPhotos(photos: Photo[], limit: number): Photo[] {
  return [...photos].sort((a, b) => b.ts - a.ts).slice(0, limit)
}

/**
 * Oldest capture time an age filter admits.
 *
 * @param age - the filter
 * @param nowS - current unix seconds
 * @returns unix seconds, or null for 'all'
 */
export function ageCutoff(age: PhotoAge, nowS: number): number | null {
  if (age === 'all') return null
  return nowS - (age === '24h' ? 24 * SECONDS_PER_HOUR : 7 * SECONDS_PER_DAY)
}

/**
 * Colour of a camera's photos: the one photo colour, the same for every camera.
 *
 * @param _camera - camera name (unused, cameras are not told apart by colour)
 * @returns the photo marker colour
 */
export function cameraColor(_camera: string): string {
  return PHOTO_COLOR
}
