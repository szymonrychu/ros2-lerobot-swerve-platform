/**
 * The arm calibration the 3D arm views draw with: the accepted arm to camera calibration (GET /api/arm/calibration,
 * armCalibrationApi.ts) when there is one, otherwise the tab's configured mount (client.yml arm_offset, no yaw) and no
 * joint offsets, the configured values the views always drew with.
 *
 * Joint offsets: urdf_angle = measured_angle + offset (mcp_server and filter_node apply them the same way). The views
 * pose the URDF in URDF angles (toUrdfJointStates) and the draggable arm turns its URDF angles back into measured
 * commands (toMeasuredPositions) before publishing.
 */
import type { ArmMount } from '../grasp/grasp'
import { DEFAULT_ARM_OFFSET, mountYaw } from '../grasp/grasp'
import { rosToThree, rosYawToThreeY, type ThreeTuple } from '../map3d/coords'
import type { AcceptedArmCalibration } from './armCalibrationApi'

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
  /** 'accepted' for an accepted calibration, 'config' for the tab's configured mount. */
  source: 'accepted' | 'config'
  /** Session and result of an accepted calibration, null for the configured values. */
  accepted: { session: string; result: string; accepted_at: string } | null
}

const NO_OFFSETS: Readonly<Record<string, number>> = Object.freeze({})

/**
 * The configured calibration of a tab: its arm_offset (or the measured default) with no yaw and no joint offsets.
 *
 * @param armOffset - tab arm_offset [x, y, z] metres, undefined for the default mount
 * @returns the configured calibration
 */
export function configuredArmCalibration(armOffset?: readonly [number, number, number]): ArmCalibration {
  const [x, y, z] = armOffset ?? DEFAULT_ARM_OFFSET
  return { mount: [x, y, z, 0], jointOffsets: NO_OFFSETS, source: 'config', accepted: null }
}

/**
 * The calibration to draw with: the accepted one when there is one, otherwise the tab's configured mount.
 *
 * @param armOffset - tab arm_offset [x, y, z] metres, undefined for the default mount
 * @param accepted - accepted calibration from the backend, null when nothing is accepted
 * @returns the calibration
 */
export function effectiveArmCalibration(
  armOffset: readonly [number, number, number] | undefined,
  accepted: AcceptedArmCalibration | null,
): ArmCalibration {
  if (accepted === null) return configuredArmCalibration(armOffset)
  const b = accepted.base_in_base_link
  return {
    mount: [b.x, b.y, b.z, b.yaw],
    jointOffsets: { ...accepted.joint_offsets_rad },
    source: 'accepted',
    accepted: { session: accepted.session, result: accepted.result, accepted_at: accepted.accepted_at },
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
