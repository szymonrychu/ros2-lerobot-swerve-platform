/** Ground picking in the Arm tab's 3D view: the arm root is the origin, the floor is under it at -mount z. */
import type * as THREE from 'three'
import { Pose2D, Vec2 } from '../map/mapMath'
import { pickGround } from '../map3d/picking'
import type { ArmMount } from '../grasp/grasp'

/**
 * The identity pose handed to the grasp pick: a point picked in the arm view is already in base_link, so no map-to-base
 * conversion applies.
 */
export const BASE_LINK_POSE: Pose2D = { x: 0, y: 0, yaw: 0 }

/**
 * Height (three y) of the floor in the arm view, whose origin is the arm root.
 *
 * @param armOffset - arm mount in base_link (ROS x, y, z metres)
 * @returns the floor height: base_link z = 0 is -mount z below the arm root
 */
export function armViewGroundY(armOffset: ArmMount): number {
  return -armOffset[2]
}

/**
 * Convert a floor hit in the arm frame to base_link (the frames only differ by the mount translation).
 *
 * @param hit - ground point in the arm frame (ROS x, y), or null above the horizon
 * @param armOffset - arm mount in base_link (ROS x, y, z metres)
 * @returns the point in base_link, or null for a null hit
 */
export function armViewHitToBaseLink(hit: Vec2 | null, armOffset: ArmMount): Vec2 | null {
  return hit && { x: hit.x + armOffset[0], y: hit.y + armOffset[1] }
}

/**
 * The base_link floor point under a screen position in the arm view.
 *
 * @param camera - scene camera with an up-to-date world matrix
 * @param px - x in CSS px from the canvas left edge
 * @param py - y in CSS px from the canvas top edge
 * @param width - canvas width in CSS px
 * @param height - canvas height in CSS px
 * @param armOffset - arm mount in base_link (ROS x, y, z metres)
 * @returns base_link point {x, y}, or null when the pixel looks above the horizon
 */
export function pickArmViewGround(
  camera: THREE.Camera,
  px: number,
  py: number,
  width: number,
  height: number,
  armOffset: ArmMount,
): Vec2 | null {
  return armViewHitToBaseLink(pickGround(camera, px, py, width, height, armViewGroundY(armOffset)), armOffset)
}
