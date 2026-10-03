/** Ground-plane picking: a screen point to the ROS map-frame point under it. */
import * as THREE from 'three'
import { threeToRos, ThreeTuple } from './coords'
import { Vec2 } from '../map/mapMath'

const MIN_DOWNWARD = 1e-9 // rays flatter than this never reach the ground

/**
 * Canvas pixel to normalised device coordinates.
 *
 * @param px - x in CSS px from the canvas left edge
 * @param py - y in CSS px from the canvas top edge
 * @param width - canvas width in CSS px
 * @param height - canvas height in CSS px
 * @returns NDC {x, y} in [-1, 1] with y up, or null for an empty canvas
 */
export function screenToNdc(px: number, py: number, width: number, height: number): Vec2 | null {
  if (width <= 0 || height <= 0) return null
  return { x: (px / width) * 2 - 1, y: 1 - (py / height) * 2 }
}

/**
 * Intersect a three.js ray with the horizontal ground plane three y = groundY.
 *
 * @param origin - ray origin (three coordinates)
 * @param dir - ray direction (three coordinates, need not be normalised)
 * @param groundY - height of the ground plane (three y)
 * @returns the hit as a ROS map point {x, y}, or null when the ray does not go down to the plane
 */
export function rayGroundIntersection(origin: ThreeTuple, dir: ThreeTuple, groundY: number): Vec2 | null {
  if (dir[1] > -MIN_DOWNWARD) return null
  const t = (groundY - origin[1]) / dir[1]
  if (t < 0) return null
  const hit = threeToRos([origin[0] + dir[0] * t, groundY, origin[2] + dir[2] * t])
  return { x: hit.x, y: hit.y }
}

/**
 * Map-frame point under a screen position for the given camera.
 *
 * @param camera - scene camera (perspective or orthographic) with an up-to-date world matrix
 * @param px - x in CSS px from the canvas left edge
 * @param py - y in CSS px from the canvas top edge
 * @param width - canvas width in CSS px
 * @param height - canvas height in CSS px
 * @param groundY - height of the ground plane (three y), default 0
 * @returns ROS map point {x, y}, or null when the pixel looks above the horizon
 */
export function pickGround(
  camera: THREE.Camera,
  px: number,
  py: number,
  width: number,
  height: number,
  groundY = 0,
): Vec2 | null {
  const ndc = screenToNdc(px, py, width, height)
  if (!ndc) return null
  const raycaster = new THREE.Raycaster()
  raycaster.setFromCamera(new THREE.Vector2(ndc.x, ndc.y), camera)
  const { origin, direction } = raycaster.ray
  return rayGroundIntersection([origin.x, origin.y, origin.z], [direction.x, direction.y, direction.z], groundY)
}
