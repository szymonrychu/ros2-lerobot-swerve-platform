/** Placement of ground textures and camera framing in the 3D map scene (pure math, ROS map frame). */
import { MapMeta, mapPixelToWorld, Vec2 } from '../map/mapMath'

const MIN_FIT_DISTANCE = 1 // metres
const WHEEL_LINK_PATTERN = /wheel|steer/i

/** A rectangle on the ground: centre, size in metres and yaw of its width axis. */
export interface GroundPlacement {
  center: Vec2
  width: number
  height: number
  yaw: number
}

export interface Bounds {
  minX: number
  maxX: number
  minY: number
  maxY: number
}

/**
 * Ground rectangle of an OccupancyGrid image (map or costmap payload).
 *
 * @param meta - grid placement as serialized by the backend
 * @returns centre, metric size and yaw
 */
export function mapPlacement(meta: MapMeta): GroundPlacement {
  return {
    center: mapPixelToWorld(meta, meta.width / 2, meta.height / 2),
    width: meta.width * meta.resolution,
    height: meta.height * meta.resolution,
    yaw: meta.origin.yaw,
  }
}

/**
 * Axis-aligned map-frame extent of a grid.
 *
 * @param meta - grid placement
 * @returns bounds in metres
 */
export function mapBounds(meta: MapMeta): Bounds {
  const corners = [
    mapPixelToWorld(meta, 0, 0),
    mapPixelToWorld(meta, meta.width, 0),
    mapPixelToWorld(meta, 0, meta.height),
    mapPixelToWorld(meta, meta.width, meta.height),
  ]
  const xs = corners.map((p) => p.x)
  const ys = corners.map((p) => p.y)
  return { minX: Math.min(...xs), maxX: Math.max(...xs), minY: Math.min(...ys), maxY: Math.max(...ys) }
}

/**
 * Camera distance at which a ground rectangle fits a perspective view looking straight at it.
 *
 * @param spanX - width to fit (screen horizontal), metres
 * @param spanY - height to fit (screen vertical), metres
 * @param fovDeg - vertical field of view, degrees
 * @param aspect - viewport width / height
 * @param fill - fraction of the view to fill (default 0.9)
 * @returns distance in metres (at least MIN_FIT_DISTANCE)
 */
export function fitDistance(spanX: number, spanY: number, fovDeg: number, aspect: number, fill = 0.9): number {
  const tanHalf = Math.tan((fovDeg * Math.PI) / 360)
  const needY = spanY / 2 / tanHalf
  const needX = spanX / 2 / (tanHalf * Math.max(aspect, 1e-6))
  return Math.max(MIN_FIT_DISTANCE, Math.max(needX, needY) / fill)
}

/**
 * Which robot-part toggle a base URDF link belongs to.
 *
 * @param linkName - URDF link name
 * @returns 'wheels' for wheel and steering links, otherwise 'body'
 */
export function classifyBaseLink(linkName: string): 'wheels' | 'body' {
  return WHEEL_LINK_PATTERN.test(linkName) ? 'wheels' : 'body'
}
