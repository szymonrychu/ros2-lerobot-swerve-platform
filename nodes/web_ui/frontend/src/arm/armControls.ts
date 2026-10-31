import type { ServoGroupsState } from '../calibrate/servoApi'
import type { TabConfig } from '../types'

/** filter_node reports `autonomy` while the mcp_server lease taken by Take control (or the agent) is held. */
export const HELD_SOURCE = 'autonomy'
const DEFAULT_SOURCE_TOPIC = '/filter/active_source'
export const HELD_REASON = 'Arm held by the agent or Take control - Release to drag'
export const BUSY_REASON = 'Arm action in flight'
export const WAITING_REASON = 'Waiting for arm joint states'
const RELEASED_SUFFIX = '(Calibrate page)'

export type ArmControl = 'drag' | 'armHome' | 'setHome' | 'grasp'

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
}

export interface ControlState {
  disabled: boolean
  /** Why the control is disabled; empty while it is enabled. */
  reason: string
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
 * @param input - released state, in-flight flags, readiness and the active command source
 * @returns per control `{ disabled, reason }`; reason is '' while enabled
 */
export function armControlState(input: ArmControlInput): Record<ArmControl, ControlState> {
  const off = (reason: string): ControlState => ({ disabled: true, reason })
  const on: ControlState = { disabled: false, reason: '' }
  const releasedReason = input.releasedJoints.length
    ? `Arm torque is off: ${input.releasedJoints.join(', ')} released ${RELEASED_SUFFIX}`
    : `Arm torque is off ${RELEASED_SUFFIX}`
  const released = input.released ? off(releasedReason) : null
  const busy = input.busy ? off(BUSY_REASON) : null
  const drag =
    released ??
    (input.activeSource === HELD_SOURCE ? off(HELD_REASON) : null) ??
    (input.armReady ? on : off(WAITING_REASON))
  return {
    drag,
    armHome: released ?? busy ?? on,
    setHome: released ?? busy ?? on,
    grasp: released ?? busy ?? (input.graspBusy ? off(BUSY_REASON) : on),
  }
}
