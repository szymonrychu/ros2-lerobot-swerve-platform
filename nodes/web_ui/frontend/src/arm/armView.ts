import { Vector3, type Camera, type Matrix4, type Object3D } from 'three'
import type { URDFRobot } from 'urdf-loader'

/** Camera mode of the Arm tab 3D view. */
export type ArmCameraMode = 'side-left' | 'side-right' | 'above'

/** Camera pose in the three.js world. */
export interface ArmCameraPose {
  position: Vector3
  target: Vector3
  up: Vector3
}

/** Arm tab glued camera modes in button order. */
export const ARM_CAMERA_MODES: readonly ArmCameraMode[] = ['side-left', 'side-right', 'above']

/**
 * Directions in the shoulder_link frame of so101_arm.urdf (the frame after shoulder_pan). The shoulder_pan origin
 * rotates base_link by (pi, 0, -pi), so shoulder x = -arm forward, shoulder y = robot left, shoulder z = -up.
 */
const FORWARD_SHOULDER = new Vector3(-1, 0, 0)
const LEFT_SHOULDER = new Vector3(0, 1, 0)
const UP_SHOULDER = new Vector3(0, 0, -1)

/** Distance of the glued side and above cameras from the look-at point, metres. */
export const SIDE_CAMERA_DISTANCE_M = 0.55
/** The look-at point sits this far ahead of the shoulder along the arm, metres. */
export const LOOK_AHEAD_M = 0.15

/**
 * Camera pose for the Arm tab views.
 *
 * The matrix is the shoulder_link world matrix read from the loaded robot, so the URDF-to-three rotation
 * (coords.ts URDF_TO_THREE_ROTATION_X) of the robot root is already inside it. The poses are built from shoulder_link
 * axes only, so they turn with shoulder_pan: the arm keeps its place on screen.
 *
 * @param mode - camera mode
 * @param shoulderWorld - shoulder_link world matrix (three space)
 * @returns camera position, look-at target and up vector in the three.js world
 */
export function armCameraPose(mode: ArmCameraMode, shoulderWorld: Matrix4): ArmCameraPose {
  const world = (local: Vector3): Vector3 => local.clone().applyMatrix4(shoulderWorld)
  const direction = (local: Vector3): Vector3 => local.clone().transformDirection(shoulderWorld)
  const lookAt = FORWARD_SHOULDER.clone().multiplyScalar(LOOK_AHEAD_M)
  if (mode === 'above') {
    const position = lookAt.clone().addScaledVector(UP_SHOULDER, SIDE_CAMERA_DISTANCE_M)
    return { position: world(position), target: world(lookAt), up: direction(FORWARD_SHOULDER) }
  }
  const side = mode === 'side-left' ? 1 : -1
  const position = lookAt.clone().addScaledVector(LEFT_SHOULDER, side * SIDE_CAMERA_DISTANCE_M)
  return { position: world(position), target: world(lookAt), up: direction(UP_SHOULDER) }
}

/**
 * Link or frame of the arm by name (urdf-loader keeps fixed frames in `frames`).
 *
 * @param robot - loaded URDF robot
 * @param name - link name
 * @returns the object, or undefined
 */
export function findLink(robot: URDFRobot, name: string): Object3D | undefined {
  const frames = (robot as unknown as { frames?: Record<string, Object3D> }).frames
  return frames?.[name] ?? robot.links[name]
}

/**
 * Glue the camera to the arm: reads the shoulder_link world matrix of the loaded robot and sets camera position, up
 * and the controls target from armCameraPose. Call it in the same frame as setJointValue.
 *
 * @param camera - camera to move
 * @param controlsTarget - orbit controls target, set to the look-at point
 * @param robot - loaded arm robot (world matrices are refreshed here)
 * @param mode - glued camera mode
 * @returns true when the camera was updated, false when shoulder_link is missing
 */
export function applyArmCamera(camera: Camera, controlsTarget: Vector3, robot: URDFRobot, mode: ArmCameraMode): boolean {
  robot.updateWorldMatrix(true, true)
  const shoulder = findLink(robot, 'shoulder_link')
  if (!shoulder) return false
  const pose = armCameraPose(mode, shoulder.matrixWorld)
  camera.position.copy(pose.position)
  camera.up.copy(pose.up)
  controlsTarget.copy(pose.target)
  camera.lookAt(pose.target)
  return true
}
