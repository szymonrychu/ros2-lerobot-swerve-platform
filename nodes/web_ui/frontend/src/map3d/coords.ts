/**
 * ROS <-> three.js coordinate conversion used by every object in the 3D map scene.
 *
 * ROS (REP 103) is right-handed z-up: x forward/east, y left/north, z up. three.js is right-handed y-up.
 * A ROS point (x, y, z) is drawn at three (x, z, -y). The mapping is a proper rotation (-90 deg about x),
 * so a ROS yaw about +z is the same angle about three +y.
 */

/** A point in a ROS frame (metres); z defaults to 0 for map-plane points. */
export interface RosPoint {
  x: number
  y: number
  z?: number
}

/** A three.js position tuple [x, y, z]. */
export type ThreeTuple = [number, number, number]

/** Rotation applied to a URDF (z-up) root so it stands upright in the y-up scene. */
export const URDF_TO_THREE_ROTATION_X = -Math.PI / 2

/**
 * Convert a ROS point to a three.js position.
 *
 * @param p - ROS point (z optional, default 0)
 * @param lift - extra height in metres added on top of z (three y), e.g. to stack ground layers
 * @returns three position [x, z + lift, -y]
 */
export function rosToThree(p: RosPoint, lift = 0): ThreeTuple {
  // `0 - p.y` instead of `-p.y` so y = 0 maps to +0 (toEqual distinguishes -0).
  return [p.x, (p.z ?? 0) + lift, 0 - p.y]
}

/**
 * Convert a three.js position back to a ROS point.
 *
 * @param t - three position [x, y, z]
 * @returns ROS point {x, y: -z, z: y}
 */
export function threeToRos(t: ThreeTuple): Required<RosPoint> {
  return { x: t[0], y: 0 - t[2], z: t[1] }
}

/**
 * ROS yaw (about +z) as a three.js rotation about +y.
 *
 * @param yaw - ROS yaw in radians
 * @returns rotation.y in radians
 */
export function rosYawToThreeY(yaw: number): number {
  return yaw
}

/**
 * three.js rotation about +y as a ROS yaw (about +z).
 *
 * @param rotationY - three rotation.y in radians
 * @returns ROS yaw in radians
 */
export function threeYToRosYaw(rotationY: number): number {
  return rotationY
}
