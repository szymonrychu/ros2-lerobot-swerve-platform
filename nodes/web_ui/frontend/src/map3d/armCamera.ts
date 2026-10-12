import { Euler, Matrix4, Quaternion, Vector3 } from 'three'

/** Camera mode of the Arm tab 3D view. */
export type ArmCameraMode = 'side-left' | 'side-right' | 'above' | 'gripper'

/** Camera mount relative to its parent link, as in client.yml (URDF origin convention). */
export interface ArmCameraMount {
  /** Translation [x, y, z] in metres. */
  xyz: readonly [number, number, number]
  /** Fixed-axis roll, pitch, yaw in radians (R = Rz(yaw) Ry(pitch) Rx(roll)). */
  rpy: readonly [number, number, number]
}

/** Camera pose in the three.js world. */
export interface ArmCameraPose {
  position: Vector3
  target: Vector3
  up: Vector3
}

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
 * Both matrices are world matrices read from the loaded robot, so the URDF-to-three rotation
 * (coords.ts URDF_TO_THREE_ROTATION_X) of the robot root is already inside them. Side and above poses are built
 * from shoulder_link axes only, so they turn with shoulder_pan: the arm keeps its place on screen. The gripper pose
 * is gripper_link composed with the mount, looking along the camera optical axis (mount body +x, +z up).
 *
 * @param mode - camera mode
 * @param shoulderWorld - shoulder_link world matrix (three space)
 * @param gripperWorld - gripper_link world matrix (three space)
 * @param mount - gripper camera mount relative to gripper_link
 * @returns camera position, look-at target and up vector in the three.js world
 */
export function armCameraPose(
  mode: ArmCameraMode,
  shoulderWorld: Matrix4,
  gripperWorld: Matrix4,
  mount: ArmCameraMount,
): ArmCameraPose {
  if (mode === 'gripper') {
    const rotation = new Matrix4().makeRotationFromQuaternion(
      new Quaternion().setFromEuler(new Euler(mount.rpy[0], mount.rpy[1], mount.rpy[2], 'ZYX')),
    )
    const camera = gripperWorld.clone().multiply(rotation.setPosition(mount.xyz[0], mount.xyz[1], mount.xyz[2]))
    const position = new Vector3().setFromMatrixPosition(camera)
    const axis = (x: number, y: number, z: number): Vector3 => new Vector3(x, y, z).transformDirection(camera)
    return { position, target: position.clone().add(axis(1, 0, 0)), up: axis(0, 0, 1) }
  }
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
