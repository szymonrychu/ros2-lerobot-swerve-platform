/**
 * The arm calibration the 3D arm views draw with, always the one the backend serves (GET /api/arm/calibration,
 * armCalibrationApi.ts): the accepted arm to camera calibration, or with nothing accepted the configured client.yml
 * values (web_ui arm_calibration, the joint offsets and arm mount mcp_server and filter_node use). The views have no
 * mount of their own: until the backend answers they draw no arm.
 *
 * Joint offsets: urdf_angle = measured_angle + offset (mcp_server and filter_node apply them the same way). The views
 * pose the URDF in URDF angles (toUrdfJointStates) and the draggable arm turns its URDF angles back into measured
 * commands (toMeasuredPositions) before publishing.
 */
import type { ArmMount } from '../grasp/grasp'
import { mountYaw } from '../grasp/grasp'
import { rosToThree, rosYawToThreeY, type ThreeTuple } from '../map3d/coords'
import type { ArmCalibrationSource, ServedArmCalibration } from './armCalibrationApi'

/** Joint states as the bridge forwards them (parallel name / position arrays). */
export interface JointStates {
  name?: string[]
  position?: number[]
}

/** The calibration a view draws with. */
export interface ArmCalibration {
  /** Arm mount in base_link [x, y, z, yaw] (m, rad). */
  mount: ArmMount
  /** Zero offset per joint (rad, urdf = measured + offset); a joint without one has 0. */
  jointOffsets: Readonly<Record<string, number>>
  /** 'accepted' for an accepted calibration, 'configured' for the configured client.yml values. */
  source: ArmCalibrationSource
  /** Session and result of an accepted calibration, null for the configured values. */
  accepted: { session: string; result: string; accepted_at: string } | null
}

/**
 * The calibration to draw with, from the one the backend serves.
 *
 * @param served - calibration served by GET /api/arm/calibration (accepted or configured)
 * @returns the calibration (its own copies of the offsets and the identity)
 */
export function toArmCalibration(served: ServedArmCalibration): ArmCalibration {
  const b = served.base_in_base_link
  return {
    mount: [b.x, b.y, b.z, b.yaw],
    jointOffsets: { ...served.joint_offsets_rad },
    source: served.source,
    accepted: served.accepted === null ? null : { ...served.accepted },
  }
}

/**
 * Measured joint states as URDF angles (urdf = measured + offset); the input is returned as is without offsets.
 *
 * @param states - measured joint states, undefined before the first message
 * @param offsets - joint offsets (rad)
 * @returns joint states in URDF angles
 */
export function toUrdfJointStates(
  states: JointStates | undefined,
  offsets: Readonly<Record<string, number>>,
): JointStates | undefined {
  const names = states?.name
  if (!states || !names || !states.position || Object.keys(offsets).length === 0) return states
  return { ...states, position: states.position.map((p, i) => p + (offsets[names[i]] ?? 0)) }
}

/**
 * URDF joint angles back to measured positions (measured = urdf - offset), in the names' order.
 *
 * @param names - joint names
 * @param urdf - URDF angle per joint name (rad); a missing joint is 0
 * @param offsets - joint offsets (rad)
 * @returns measured positions aligned with names
 */
export function toMeasuredPositions(
  names: readonly string[],
  urdf: Readonly<Record<string, number>>,
  offsets: Readonly<Record<string, number>>,
): number[] {
  return names.map((n) => (urdf[n] ?? 0) - (offsets[n] ?? 0))
}

/**
 * Where the arm's URDF root sits in the robot (base_link) group of a three.js scene.
 *
 * @param mount - arm mount in base_link
 * @returns the group position and rotation (the mount yaw about three +y)
 */
export function armScenePlacement(mount: ArmMount): { position: ThreeTuple; rotation: ThreeTuple } {
  return { position: rosToThree({ x: mount[0], y: mount[1], z: mount[2] }), rotation: [0, rosYawToThreeY(mountYaw(mount)), 0] }
}
