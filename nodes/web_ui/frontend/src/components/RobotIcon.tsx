import { useTheme } from '@mui/material/styles'
import { BaseJointMessage } from '../map3d/baseJoints'
import { robotIconGeometry } from './robotIconGeometry'

interface Props {
  /** Latest /swerve_drive/joint_states message, undefined before the first one. */
  joints: BaseJointMessage | undefined
  /** Rendered height in px. */
  height?: number
}

const ROLLER_RADIUS_FRACTION = 0.3
const STROKE_WIDTH = 1.5
const INACTIVE_OPACITY = 0.4

/** Top view of the robot (front up): footprint outline, front marker and four steering rollers, greyed until data. */
export function RobotIcon({ joints, height = 40 }: Props) {
  const theme = useTheme()
  const g = robotIconGeometry(joints)
  const [, , vw, vh] = g.viewBox.split(' ').map(Number)
  const roller = g.active ? theme.palette.primary.main : theme.palette.text.disabled
  const outline = g.active ? theme.palette.text.secondary : theme.palette.text.disabled
  const m = g.frontMarker
  return (
    <svg
      role="img"
      aria-label={g.active ? 'Robot wheel steering' : 'Robot wheel steering, no data yet'}
      viewBox={g.viewBox}
      height={height}
      width={(height * vw) / vh}
      style={{ flexShrink: 0, opacity: g.active ? 1 : INACTIVE_OPACITY }}
    >
      <rect {...g.footprint} rx={4} fill="none" stroke={outline} strokeWidth={STROKE_WIDTH} />
      <polygon points={`${m.tipX},${m.tipY} ${m.tipX - m.halfWidth},${m.baseY} ${m.tipX + m.halfWidth},${m.baseY}`} fill={outline} />
      {g.rollers.map((r) => (
        <rect
          key={r.id}
          data-testid={`roller-${r.id}`}
          x={r.cx - r.width / 2}
          y={r.cy - r.length / 2}
          width={r.width}
          height={r.length}
          rx={r.width * ROLLER_RADIUS_FRACTION * 2}
          fill={roller}
          transform={`rotate(${r.rotationDeg} ${r.cx} ${r.cy})`}
        />
      ))}
    </svg>
  )
}
