/** Joint values of the head URDF from /head/joint_states. */

/** Names of the pan and tilt joints, in the order returned. */
export const HEAD_JOINT_NAMES = ['head_pan', 'head_tilt'] as const

/** Subset of sensor_msgs/JointState used here. */
export interface HeadJointMessage {
  name?: string[]
  position?: number[]
}

/** Joint values ready for RobotModel: parallel name / position arrays. */
export interface HeadJointValues {
  name: string[]
  position: number[]
}

/**
 * Map a head JointState message to URDF joint values.
 *
 * @param msg - the latest /head/joint_states message, or undefined before one arrived
 * @returns head_pan and head_tilt positions in rad, or undefined when there is no message or either value is missing
 *   or non-finite (the head is then not drawn: no placeholder pose)
 */
export function headJointValues(msg: HeadJointMessage | undefined): HeadJointValues | undefined {
  if (!msg?.name || !msg.position) return undefined
  const position: number[] = []
  for (const joint of HEAD_JOINT_NAMES) {
    const p = msg.position[msg.name.indexOf(joint)]
    if (typeof p !== 'number' || !Number.isFinite(p)) return undefined
    position.push(p)
  }
  return { name: [...HEAD_JOINT_NAMES], position }
}
