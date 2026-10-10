/**
 * Geometry of the status bar mini robot: top view, front up, footprint and four rollers rotated by the steering
 * angles of /swerve_drive/joint_states (matched by joint name).
 */
import {
  BASE_LENGTH_M,
  BASE_WIDTH_M,
  BaseJointMessage,
  STEER_JOINT_SUFFIX,
  SWERVE_HALF_LENGTH_M,
  SWERVE_HALF_WIDTH_M,
  SWERVE_MODULES,
  WHEEL_RADIUS_M,
  WHEEL_WIDTH_M,
} from '../map3d/baseJoints'

/** Icon units per metre: the footprint is drawn BASE_LENGTH_M * ICON_SCALE tall, centred on the origin. */
export const ICON_SCALE = 100
/** Front marker: triangle height and half width as a fraction of the footprint width. */
const MARKER_HEIGHT_FRACTION = 0.18
const MARKER_HALF_WIDTH_FRACTION = 0.14
/** Gap between the footprint edge and the marker tip, and padding around the drawing, in icon units. */
const MARKER_GAP = 2
const VIEW_PADDING = 4

export interface IconRoller {
  id: string
  /** Centre in icon units (x right, y down, origin at the base centre). */
  cx: number
  cy: number
  /** Extent along the roller rolling direction (wheel diameter) and across it (wheel width). */
  length: number
  width: number
  /** SVG rotation about the centre, degrees; a positive (CCW from above) steering angle turns it CCW. */
  rotationDeg: number
}

export interface RobotIconGeometry {
  viewBox: string
  footprint: { x: number; y: number; width: number; height: number }
  rollers: IconRoller[]
  /** Triangle pointing at the front: tip above the footprint, base on the footprint's top edge. */
  frontMarker: { tipX: number; tipY: number; baseY: number; halfWidth: number }
  /** True once at least one steering joint arrived; the icon is greyed before. */
  active: boolean
}

/**
 * Build the icon geometry from the latest swerve joint states.
 *
 * @param joints - JointState-like message, or undefined before the first one (rollers straight, inactive)
 * @returns footprint, rollers (fl, fr, rl, rr), front marker and the active flag in icon units
 */
export function robotIconGeometry(joints: BaseJointMessage | undefined): RobotIconGeometry {
  const height = BASE_LENGTH_M * ICON_SCALE
  const width = BASE_WIDTH_M * ICON_SCALE
  let active = false
  const rollers = SWERVE_MODULES.map(({ id, xSign, ySign }) => {
    const idx = joints?.name?.indexOf(`${id}${STEER_JOINT_SUFFIX}`) ?? -1
    const angle = idx >= 0 ? joints?.position?.[idx] : undefined
    const steering = typeof angle === 'number' && Number.isFinite(angle)
    if (steering) active = true
    return {
      id,
      // Robot +x (forward) is icon up, robot +y (left) is icon left.
      cx: -ySign * SWERVE_HALF_WIDTH_M * ICON_SCALE,
      cy: -xSign * SWERVE_HALF_LENGTH_M * ICON_SCALE,
      length: 2 * WHEEL_RADIUS_M * ICON_SCALE,
      width: WHEEL_WIDTH_M * ICON_SCALE,
      rotationDeg: steering ? -(angle * 180) / Math.PI : 0,
    }
  })
  const markerHeight = width * MARKER_HEIGHT_FRACTION
  const top = -height / 2
  const halfW = width / 2 + VIEW_PADDING
  const halfH = height / 2 + VIEW_PADDING
  const tipY = top - MARKER_GAP - markerHeight
  const viewTop = Math.min(-halfH, tipY - VIEW_PADDING)
  return {
    viewBox: `${-halfW} ${viewTop} ${2 * halfW} ${halfH - viewTop}`,
    footprint: { x: -width / 2, y: top, width, height },
    rollers,
    frontMarker: { tipX: 0, tipY, baseY: top - MARKER_GAP, halfWidth: width * MARKER_HALF_WIDTH_FRACTION },
    active,
  }
}
