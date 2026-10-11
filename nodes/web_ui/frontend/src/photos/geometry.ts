/** Pure geometry and filters of the map photo layer (map frame, metres and radians). */
import type { Photo } from './api'

/** Length of a viewing-direction wedge, m. */
export const WEDGE_LENGTH_M = 0.35
/** Display-only wedge width when the field of view is unknown (60 deg; not data, the wedge is dashed). */
export const DEFAULT_HFOV_RAD = Math.PI / 3
/** Most photos the map layer draws. */
export const LAYER_MAX_PHOTOS = 1000
const SECONDS_PER_HOUR = 3600
const SECONDS_PER_DAY = 86400

/** Age filter of the layer. */
export type PhotoAge = '24h' | '7d' | 'all'
export const PHOTO_AGES: readonly PhotoAge[] = ['24h', '7d', 'all']
export const PHOTO_AGE_LABELS: Record<PhotoAge, string> = { '24h': '24 h', '7d': '7 d', all: 'All' }

const CAMERA_PALETTE = ['#4cc9f0', '#f72585', '#b5e48c', '#ffd166', '#c77dff', '#ff9f1c', '#2ec4b6', '#e63946'] as const

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
 * Stable colour of a camera name.
 *
 * @param camera - camera name
 * @returns a CSS colour from a fixed palette
 */
export function cameraColor(camera: string): string {
  let h = 0
  for (const ch of camera) h = (h * 31 + ch.charCodeAt(0)) >>> 0
  return CAMERA_PALETTE[h % CAMERA_PALETTE.length]
}
