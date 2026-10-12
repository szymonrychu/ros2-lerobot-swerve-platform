import type { Camera, Object3D, Vector3 } from 'three'
import type { URDFRobot } from 'urdf-loader'
import { armCameraPose, type ArmCameraMode, type ArmCameraMount } from '../map3d/armCamera'

/** Arm tab glued camera modes in button order. */
export const ARM_CAMERA_MODES: readonly ArmCameraMode[] = ['side-left', 'side-right', 'above', 'gripper']

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
 * Glue the camera to the arm: reads the shoulder_link and gripper_link world matrices of the loaded robot and sets
 * camera position, up and the controls target from armCameraPose. Call it in the same frame as setJointValue.
 *
 * @param camera - camera to move
 * @param controlsTarget - orbit controls target, set to the look-at point
 * @param robot - loaded arm robot (world matrices are refreshed here)
 * @param mode - glued camera mode
 * @param mount - gripper camera mount relative to gripper_link
 * @returns true when the camera was updated, false when a link is missing
 */
export function applyArmCamera(
  camera: Camera,
  controlsTarget: Vector3,
  robot: URDFRobot,
  mode: ArmCameraMode,
  mount: ArmCameraMount,
): boolean {
  robot.updateWorldMatrix(true, true)
  const shoulder = findLink(robot, 'shoulder_link')
  const gripper = findLink(robot, 'gripper_link')
  if (!shoulder || !gripper) return false
  const pose = armCameraPose(mode, shoulder.matrixWorld, gripper.matrixWorld, mount)
  camera.position.copy(pose.position)
  camera.up.copy(pose.up)
  controlsTarget.copy(pose.target)
  camera.lookAt(pose.target)
  return true
}
