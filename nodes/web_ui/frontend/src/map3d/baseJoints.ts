/**
 * Joint values of the swerve base URDF from /swerve_drive/joint_states.
 *
 * The steering joints (`<module>_steer`) carry an angle in rad (positive = CCW from above, 0 = straight ahead), which
 * is the URDF joint value as is. The drive joints (`<module>_drive`) are wheel-mode servos: their `position` is a
 * wrapping encoder reading, so the roller angle is instead the integral of `velocity` (rad/s, positive = forward,
 * already de-inverted by the feetech bridge) over the time between messages.
 */

/** Suffix of the continuous wheel joints in the swerve joint names. */
export const DRIVE_JOINT_SUFFIX = '_drive'

/** Longest time step integrated per message (s): a stalled stream must not spin the wheel on resume. */
export const MAX_INTEGRATION_DT_S = 0.25

/** Subset of sensor_msgs/JointState used here. */
export interface BaseJointMessage {
  name?: string[]
  position?: number[]
  velocity?: number[]
}

/** Joint values ready for RobotModel: parallel name / position arrays. */
export interface BaseJointValues {
  name: string[]
  position: number[]
}

/**
 * Wrap an angle into [-pi, pi).
 *
 * @param angle - angle in rad
 * @returns the equivalent angle in [-pi, pi)
 */
export function wrapAngle(angle: number): number {
  const twoPi = 2 * Math.PI
  return angle - twoPi * Math.floor((angle + Math.PI) / twoPi)
}

/**
 * Advance the base joint values by one joint-state message.
 *
 * @param prev - values returned for the previous message (drive angles are carried over), or undefined at start
 * @param msg - the new JointState message
 * @param dtSec - seconds since the previous message; clamped to [0, MAX_INTEGRATION_DT_S]
 * @returns steering (and any other non-drive) joint positions of the message plus integrated drive angles
 */
export function advanceBaseJoints(
  prev: BaseJointValues | undefined,
  msg: BaseJointMessage,
  dtSec: number,
): BaseJointValues {
  const dt = Math.min(Math.max(dtSec, 0), MAX_INTEGRATION_DT_S)
  const drive = new Map<string, number>()
  if (prev) {
    prev.name.forEach((n, i) => {
      if (n.endsWith(DRIVE_JOINT_SUFFIX)) drive.set(n, prev.position[i])
    })
  }
  const out: BaseJointValues = { name: [], position: [] }
  const names = msg.name ?? []
  names.forEach((n, i) => {
    if (n.endsWith(DRIVE_JOINT_SUFFIX)) {
      const v = msg.velocity?.[i]
      if (typeof v === 'number' && Number.isFinite(v)) drive.set(n, wrapAngle((drive.get(n) ?? 0) + v * dt))
      return
    }
    const p = msg.position?.[i]
    if (typeof p === 'number' && Number.isFinite(p)) {
      out.name.push(n)
      out.position.push(p)
    }
  })
  drive.forEach((angle, n) => {
    out.name.push(n)
    out.position.push(angle)
  })
  return out
}
