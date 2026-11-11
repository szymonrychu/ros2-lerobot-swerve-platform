import type { ServoGroupsState } from '../calibrate/servoApi'
import { servoCalibrationHomeReason } from '../calibrate/ZeroRestorePanel'
import type { DrivePoseState, GuardState, TabConfig } from '../types'

/** filter_node reports `autonomy` while the mcp_server lease taken by Take control (or the agent) is held. */
export const HELD_SOURCE = 'autonomy'
const DEFAULT_SOURCE_TOPIC = '/filter/active_source'
const DEFAULT_GUARD_TOPIC = '/filter/guard_state'
const DEFAULT_DRIVE_POSE_TOPIC = '/arm/drive_pose/state'
export const HELD_REASON = 'Arm held by the agent or Take control - Release to drag'
export const BUSY_REASON = 'Arm action in flight'
export const WAITING_REASON = 'Waiting for arm joint states'
export const DRIVE_POSE_WAITING_REASON = 'Waiting for the drive pose state'
const RELEASED_SUFFIX = '(Calibrate page)'

export type ArmControl = 'drag' | 'armHome' | 'setHome' | 'setDrivePose' | 'drivePose' | 'grasp'

export interface ArmControlInput {
  /** Arm torque state is fresh and released. */
  released: boolean
  /** Joint names that report torque off (see releasedJoints). */
  releasedJoints: string[]
  /** An arm request (home, set home, grasp call) is in flight. */
  busy: boolean
  /** The draggable arm has received its first joint states. */
  armReady: boolean
  /** Value of /filter/active_source, undefined before the first message. */
  activeSource: string | undefined
  /** A grasp plan or execute is running. */
  graspBusy: boolean
  /** The drive pose state has arrived, so the toggle knows whether the lift is enabled. */
  drivePoseKnown: boolean
  /**
   * servo_calibration_at while the arm group is in the servo calibration redo state (servoCalibrationRedoAt): Arm home
   * is off then, the Arm to camera flow homes the arm after its old joint offsets confirm. null or absent otherwise.
   */
  servoCalibrationAt?: string | null
}

export interface ControlState {
  disabled: boolean
  /** Why the control is disabled; empty while it is enabled. */
  reason: string
}

/**
 * Why a limp arm cannot be commanded.
 *
 * @param joints - joint names that report torque off; may be empty
 * @returns the reason text, naming the joints when known
 */
export function releasedReason(joints: string[]): string {
  return joints.length ? `Arm torque is off: ${joints.join(', ')} released ${RELEASED_SUFFIX}` : `Arm torque is off ${RELEASED_SUFFIX}`
}

/**
 * The command source filter_node reports on the arm tab's active_source_topic.
 *
 * @param tab - arm tab config (active_source_topic)
 * @param topicData - latest payload per topic
 * @returns source name, undefined before the first message
 */
export function activeSourceOf(tab: TabConfig, topicData: Record<string, unknown>): string | undefined {
  return (topicData[tab.active_source_topic ?? DEFAULT_SOURCE_TOPIC] as { data?: string } | null | undefined)?.data
}

/**
 * The mcp_server drive pose state on the tab's drive_pose_state_topic.
 *
 * @param tab - arm or map_nav tab config (drive_pose_state_topic)
 * @param topicData - latest payload per topic
 * @returns the state, undefined before the first message or after the cache was cleared
 */
export function drivePoseOf(tab: TabConfig, topicData: Record<string, unknown>): DrivePoseState | undefined {
  return (topicData[tab.drive_pose_state_topic ?? DEFAULT_DRIVE_POSE_TOPIC] as DrivePoseState | null | undefined) ?? undefined
}

/**
 * The filter_node self-collision guard state on the tab's guard_state_topic.
 *
 * @param tab - arm tab config (guard_state_topic)
 * @param topicData - latest payload per topic
 * @returns the state, undefined before the first message
 */
export function guardStateOf(tab: TabConfig, topicData: Record<string, unknown>): GuardState | undefined {
  return (topicData[tab.guard_state_topic ?? DEFAULT_GUARD_TOPIC] as GuardState | null | undefined) ?? undefined
}

/**
 * Names of the joints of a servo group whose torque reading is 0.
 *
 * @param servo - result of useServoGroups
 * @param groupId - servo group id (arm)
 * @returns joint names; empty for an enabled group, a stale or missing state, or an unknown group
 */
export function releasedJoints(servo: ServoGroupsState, groupId: string): string[] {
  const group = servo.groups.find((g) => g.id === groupId)
  if (!group || group.stale || !group.state) return []
  return Object.entries(group.state.joints)
    .filter(([, reading]) => reading === 0)
    .map(([name]) => name)
}

/**
 * Enabled state and visible reason of every arm control. Released wins over every other reason.
 *
 * @param input - released state, in-flight flags, readiness, the active command source, the drive pose state and the
 *   servo calibration redo time
 * @returns per control `{ disabled, reason }`; reason is '' while enabled
 */
export function armControlState(input: ArmControlInput): Record<ArmControl, ControlState> {
  const off = (reason: string): ControlState => ({ disabled: true, reason })
  const on: ControlState = { disabled: false, reason: '' }
  const released = input.released ? off(releasedReason(input.releasedJoints)) : null
  const busy = input.busy ? off(BUSY_REASON) : null
  const drag =
    released ??
    (input.activeSource === HELD_SOURCE ? off(HELD_REASON) : null) ??
    (input.armReady ? on : off(WAITING_REASON))
  return {
    drag,
    armHome: released ?? busy ?? (input.servoCalibrationAt ? off(servoCalibrationHomeReason(input.servoCalibrationAt)) : on),
    setHome: released ?? busy ?? on,
    setDrivePose: released ?? busy ?? on,
    drivePose: released ?? busy ?? (input.drivePoseKnown ? on : off(DRIVE_POSE_WAITING_REASON)),
    grasp: released ?? busy ?? (input.graspBusy ? off(BUSY_REASON) : on),
  }
}
