/** Ground board parts of the Camera to ground flow: the session status, the result details and the verification. */
import Box from '@mui/material/Box'
import Chip from '@mui/material/Chip'
import Table from '@mui/material/Table'
import TableBody from '@mui/material/TableBody'
import TableCell from '@mui/material/TableCell'
import TableHead from '@mui/material/TableHead'
import TableRow from '@mui/material/TableRow'
import Typography from '@mui/material/Typography'
import { CameraStream } from '../components/CameraStream'
import type {
  GroundAccept,
  GroundAccepted,
  GroundHeadChain,
  GroundReport,
  GroundSweep,
  GroundVerify,
  GroundVerifyCheck,
  GroundVerifyPose,
} from './groundBoardApi'

const M_TO_MM = 1000
/** Plain names of the head chain sources the node reports. */
const SOURCE_LABELS: Record<string, string> = {
  floor_fit: 'depth floor fit',
  head_arm: 'head from arm',
  arm_offsets: 'arm offsets',
  ground_board: 'ground board',
  urdf: 'URDF (not calibrated)',
}

/** The head camera stream of the Head tab, when this page has one. */
export interface HeadCameraPreview {
  topic: string
  /** Latest topic payload ({ jpeg_b64 } once a frame arrived). */
  data: unknown
}

const fixed = (v: number | null, digits: number) => (v === null ? '-' : v.toFixed(digits))

/**
 * Plain name of a head chain source.
 *
 * @param source - the node's source name, null when unknown
 * @returns the label
 */
export const sourceLabel = (source: string | null) => (source === null ? 'unknown' : SOURCE_LABELS[source] ?? source)

/**
 * Two head chain records name the same chain.
 *
 * @param a - a head chain record
 * @param b - another one
 * @returns true when both are present with the same source, session and result
 */
export const sameChain = (a: GroundHeadChain | null, b: GroundHeadChain | null) =>
  a !== null && b !== null && a.source === b.source && a.session === b.session && a.result === b.result

/**
 * A head chain is applied (by any fit): the ground verification and the arm to camera fit can run.
 *
 * @param chain - the head chain in effect
 * @returns true unless there is none or only the URDF
 */
export const headChainApplied = (chain: GroundHeadChain | null) => chain !== null && chain.source !== null && chain.source !== 'urdf'

/**
 * The head chain in effect and the accepted ground board result with its verification.
 *
 * @param chain - the head chain in effect
 * @param accepted - the ground board result whose chain is applied, null for another chain
 */
export function GroundAppliedView({ chain, accepted }: { chain: GroundHeadChain | null; accepted: GroundAccepted | null }) {
  const verification = accepted?.verification
  const verdict =
    !verification || verification.passed === null ? 'not verified yet' : verification.passed ? 'verification passed' : 'verification failed'
  return (
    <Box sx={{ mt: 1 }}>
      <Typography variant="body2">
        {chain === null || chain.source === null || chain.source === 'urdf'
          ? 'No head chain applied yet (the URDF values are in effect).'
          : `Head chain in effect: ${sourceLabel(chain.source)}, session ${chain.session ?? '-'}, result ${chain.result ?? '-'}.`}
      </Typography>
      {accepted && (
        <Typography variant="body2" color="text.secondary">
          {`Accepted ground board result ${accepted.result} (session ${accepted.session}, ${accepted.accepted_at}): RMS ${fixed(accepted.score.rms_px, 2)} px, held-out ${fixed(accepted.score.heldout_rms_px, 2)} px; ${verdict}.`}
        </Typography>
      )}
    </Box>
  )
}

/**
 * The head camera preview, or where to look when this page has none.
 *
 * @param preview - the head camera stream, null when the robot config has no head tab
 */
export function BoardPreview({ preview }: { preview: HeadCameraPreview | null }) {
  if (!preview) {
    return (
      <Typography variant="body2" color="text.secondary">
        No head camera preview here: check the board in the Head tab with the head homed.
      </Typography>
    )
  }
  return (
    <Box sx={{ mt: 1, height: 240, maxWidth: 480 }}>
      <CameraStream topic={preview.topic} data={preview.data} alt="head camera" />
    </Box>
  )
}

/**
 * Status of a ground session: placements done, the running placement's pose count and the located board.
 *
 * @param sweep - the session state
 */
export function GroundSweepStatus({ sweep }: { sweep: GroundSweep }) {
  return (
    <Box>
      <Typography variant="body2">{`${sweep.placements_done} of ${sweep.max_placements} placements done.`}</Typography>
      {!sweep.awaiting_placement && sweep.running && (
        <Typography variant="body2">
          {`Placement ${sweep.placement + 1} of ${sweep.max_placements}: pose ${sweep.pose} of ${sweep.total}, ${sweep.views} views, ${sweep.planes} depth planes, ${sweep.skipped} skipped - ${sweep.status}`}
        </Typography>
      )}
      {!sweep.running && (
        <Typography variant="body2">{`${sweep.views} views, ${sweep.planes} depth planes, ${sweep.skipped} skipped - ${sweep.status}`}</Typography>
      )}
      {sweep.board && (
        <Typography variant="body2" color="text.secondary">
          {`Board located at x ${sweep.board.x_m.toFixed(3)} m, y ${sweep.board.y_m.toFixed(3)} m, yaw ${sweep.board.yaw_deg.toFixed(1)} deg.`}
        </Typography>
      )}
    </Box>
  )
}

/**
 * Text of an accept answer: the applied result and the restart state of each consumer unit.
 *
 * @param answer - the accept answer
 * @returns one line for the gate step
 */
export function groundAcceptText(answer: GroundAccept): string {
  const units = answer.units.map((u) => `${u.name} ${u.restarted ? 'restarted' : 'not restarted'} (${u.active_state})`)
  return `Applied result ${answer.head_chain.result} (ground board).${units.length ? ` Units: ${units.join(', ')}.` : ''}`
}

/**
 * Details of a ground board fit next to its gate: the fitted board placements and the head poses (held out or not).
 *
 * @param report - the report of a done result
 */
export function GroundResultDetails({ report }: { report: GroundReport }) {
  const heldOut = report.poses.filter((p) => p.held_out).length
  return (
    <Box sx={{ mt: 1 }}>
      <Box sx={{ overflowX: 'auto' }}>
        <Table size="small" aria-label="Board placements">
          <TableHead>
            <TableRow>
              {['Placement', 'x (m)', 'y (m)', 'yaw (deg)', 'Sigma x, y (mm)', 'Sigma yaw (deg)', 'Views', 'RMS (px)'].map((h) => (
                <TableCell key={h}>{h}</TableCell>
              ))}
            </TableRow>
          </TableHead>
          <TableBody>
            {report.placements.map((p) => (
              <TableRow key={p.placement}>
                <TableCell>{p.placement + 1}</TableCell>
                <TableCell>{fixed(p.x_m, 3)}</TableCell>
                <TableCell>{fixed(p.y_m, 3)}</TableCell>
                <TableCell>{fixed(p.yaw_deg, 1)}</TableCell>
                <TableCell>{`${fixed(p.sigma_x_mm, 2)}, ${fixed(p.sigma_y_mm, 2)}`}</TableCell>
                <TableCell>{fixed(p.sigma_yaw_deg, 3)}</TableCell>
                <TableCell>{p.views}</TableCell>
                <TableCell>{fixed(p.rms_px, 2)}</TableCell>
              </TableRow>
            ))}
          </TableBody>
        </Table>
      </Box>
      <Typography variant="body2" color="text.secondary">
        {`${report.poses.length} head poses, ${heldOut} held out; depth cross-check on ${report.cross_check.planes} planes at ${report.cross_check.poses} poses.`}
      </Typography>
    </Box>
  )
}

/** How a verification check is shown: label, scale to the unit and decimals. */
const CHECK_FORMATS: Record<string, { label: string; scale: number; unit: string; digits: number }> = {
  height_m: { label: 'height', scale: M_TO_MM, unit: 'mm', digits: 1 },
  plane_vs_base_deg: { label: 'plane vs base', scale: 1, unit: 'deg', digits: 2 },
  obstacle_points: { label: 'obstacle points', scale: 1, unit: '', digits: 0 },
  drop_points: { label: 'drop points', scale: 1, unit: '', digits: 0 },
  samples: { label: 'samples', scale: 1, unit: '', digits: 0 },
  plane: { label: 'plane samples', scale: 1, unit: '', digits: 0 },
}

/**
 * Text of one verification check: its median value against its limit and whether it passed.
 *
 * @param c - the check
 * @returns e.g. "height 3.0 mm (limit 10.0 mm) ok"
 */
export function verifyCheckText(c: GroundVerifyCheck): string {
  const f = CHECK_FORMATS[c.name] ?? { label: c.name, scale: 1, unit: '', digits: 3 }
  const show = (v: number | null) => (v === null ? '-' : `${(v * f.scale).toFixed(f.digits)}${f.unit ? ` ${f.unit}` : ''}`)
  return `${f.label} ${show(c.value)} (limit ${show(c.limit)}) ${c.ok ? 'ok' : 'failed'}`
}

const angle = (pan: number, tilt: number) => `pan ${pan.toFixed(1)}, tilt ${tilt.toFixed(1)}`

function VerifyRow({ pose }: { pose: GroundVerifyPose }) {
  return (
    <TableRow>
      <TableCell>{angle(pose.pan_deg, pose.tilt_deg)}</TableCell>
      <TableCell>{pose.reached ? angle(pose.reached.pan_deg, pose.reached.tilt_deg) : '-'}</TableCell>
      <TableCell>{pose.samples}</TableCell>
      <TableCell>
        <Chip size="small" label={pose.passed ? 'Passed' : 'Failed'} color={pose.passed ? 'success' : 'error'} />
      </TableCell>
      <TableCell>
        {pose.error && (
          <Typography variant="body2" color="error">
            {pose.error}
          </Typography>
        )}
        {pose.checks.map((c) => (
          <Typography key={c.name} variant="body2" color={c.ok ? 'text.primary' : 'error'}>
            {verifyCheckText(c)}
          </Typography>
        ))}
      </TableCell>
    </TableRow>
  )
}

/**
 * A ground verification: its status, the verdict once every pose was judged and every judged pose with its checks.
 *
 * @param verify - the verification state (or a stored verification)
 * @param running - the verification runs now
 * @param previous - it verified another head chain than the one in effect (shown, but it does not count)
 */
export function GroundVerifyView({ verify, running, previous = false }: { verify: GroundVerify; running: boolean; previous?: boolean }) {
  return (
    <Box sx={{ mt: 1, opacity: previous ? 0.6 : 1 }}>
      {previous && (
        <Typography variant="body2" color="text.secondary">
          This verification was of a previous head chain: verify the chain in effect again.
        </Typography>
      )}
      {verify.status !== 'idle' && (
        <Typography variant="body2">{`Verification ${verify.status.replace(/_/g, ' ')}: pose ${verify.pose} of ${verify.total}${running ? '' : '.'}`}</Typography>
      )}
      {verify.passed !== null && (
        <Chip
          size="small"
          sx={{ mt: 1 }}
          label={verify.passed ? 'Verification passed' : 'Verification failed'}
          color={verify.passed ? 'success' : 'error'}
        />
      )}
      {verify.error && (
        <Typography variant="body2" color="error">
          {verify.error}
        </Typography>
      )}
      {verify.poses.length > 0 && (
        <Box sx={{ overflowX: 'auto' }}>
          <Table size="small" sx={{ mt: 1 }} aria-label="Ground verification poses">
            <TableHead>
              <TableRow>
                {['Pose', 'Reached', 'Samples', 'Result', 'Checks (median vs limit)'].map((h) => (
                  <TableCell key={h}>{h}</TableCell>
                ))}
              </TableRow>
            </TableHead>
            <TableBody>
              {verify.poses.map((p) => (
                <VerifyRow key={p.pose} pose={p} />
              ))}
            </TableBody>
          </Table>
        </Box>
      )}
    </Box>
  )
}
