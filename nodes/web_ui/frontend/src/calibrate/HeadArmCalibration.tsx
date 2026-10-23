import { useCallback, useEffect, useState } from 'react'
import Alert from '@mui/material/Alert'
import Box from '@mui/material/Box'
import Button from '@mui/material/Button'
import Chip from '@mui/material/Chip'
import Dialog from '@mui/material/Dialog'
import DialogActions from '@mui/material/DialogActions'
import DialogContent from '@mui/material/DialogContent'
import DialogContentText from '@mui/material/DialogContentText'
import DialogTitle from '@mui/material/DialogTitle'
import LinearProgress from '@mui/material/LinearProgress'
import Table from '@mui/material/Table'
import TableBody from '@mui/material/TableBody'
import TableCell from '@mui/material/TableCell'
import TableHead from '@mui/material/TableHead'
import TableRow from '@mui/material/TableRow'
import Typography from '@mui/material/Typography'
import { TOUCH_TARGET_PX } from '../theme'
import type { AppliedHeadChain, FloorApplied, FloorApiResult } from './headFloorApi'
import {
  HeadArmResult,
  HeadArmSequence,
  HeadArmSession,
  allowBigJoints,
  applyHeadArm,
  boardAttached,
  computeHeadArm,
  getHeadArmState,
  listHeadArmSessions,
  moveToBase,
  park,
  stopSequence,
} from './headArmApi'

/** State poll period while a sequence runs: 1 Hz. */
export const ARM_POLL_MS = 1000

const M_TO_MM = 1000
const BASE_JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']
const PHASE_IDLE = 'idle'
const PHASE_WAITING_BOARD = 'waiting_board'
const PHASE_WAITING_BIG = 'waiting_big_joints'
const PHASE_HOLDING = 'holding'

type Dialog = 'base' | 'park' | 'parkBoard' | null

interface ParamRow {
  label: string
  key: string
  scale: number
  digits: number
}

const PARAM_ROWS: ParamRow[] = [
  { label: 'Mount x (mm)', key: 'mount_x_m', scale: M_TO_MM, digits: 2 },
  { label: 'Mount y (mm)', key: 'mount_y_m', scale: M_TO_MM, digits: 2 },
  { label: 'Mount z (mm)', key: 'mount_z_m', scale: M_TO_MM, digits: 2 },
  { label: 'Mount roll (deg)', key: 'mount_roll_deg', scale: 1, digits: 3 },
  { label: 'Mount pitch (deg)', key: 'mount_pitch_deg', scale: 1, digits: 3 },
  { label: 'Mount yaw (deg)', key: 'mount_yaw_deg', scale: 1, digits: 3 },
  { label: 'Tilt zero (deg)', key: 'tilt_zero_deg', scale: 1, digits: 3 },
  { label: 'Tilt roll (deg)', key: 'tilt_roll_deg', scale: 1, digits: 3 },
]

const signed = (v: number, digits: number) => `${v >= 0 ? '+' : ''}${v.toFixed(digits)}`

function ResultView({ result }: { result: HeadArmResult }) {
  const corr = result.max_correlation
  return (
    <Box sx={{ mt: 1 }}>
      <Chip label={result.passed ? 'Passed' : 'Failed'} color={result.passed ? 'success' : 'error'} size="small" />
      {result.error && (
        <Alert severity="error" sx={{ mt: 1 }}>
          {result.error}
        </Alert>
      )}
      {result.failed_limits.map((l) => (
        <Alert key={l} severity="error" sx={{ mt: 1 }}>
          {l}
        </Alert>
      ))}
      <Typography variant="body2" sx={{ mt: 1 }}>
        {`RMS ${result.rms_px.toFixed(2)} px. Held-out RMS ${result.holdout_rms_px.toFixed(2)} px. Held-out board error ${result.holdout_board_mm.toFixed(2)} mm.`}
      </Typography>
      <Typography variant="body2">
        {`${result.views} views, ${result.arm_poses} arm poses, pan span ${result.pan_span_deg.toFixed(1)} deg, tilt span ${result.tilt_span_deg.toFixed(1)} deg`}
        {corr ? `, largest correlation ${corr.value.toFixed(3)} (${corr.pair.join(' / ')})` : ''}
      </Typography>
      <Table size="small" sx={{ mt: 1 }}>
        <TableHead>
          <TableRow>
            {['Parameter', 'Start', 'Fitted', 'Delta', 'Sigma', 'Floor', 'Floor delta'].map((h) => (
              <TableCell key={h}>{h}</TableCell>
            ))}
          </TableRow>
        </TableHead>
        <TableBody>
          {PARAM_ROWS.map((row) => {
            const p = result.params[row.key]
            if (!p) return null
            const d = row.digits
            const f = (v: number) => (v * row.scale).toFixed(d)
            return (
              <TableRow key={row.key}>
                <TableCell>{row.label}</TableCell>
                <TableCell>{f(p.start)}</TableCell>
                <TableCell>{f(p.value)}</TableCell>
                <TableCell>{signed((p.value - p.start) * row.scale, d)}</TableCell>
                <TableCell>{f(p.sigma)}</TableCell>
                <TableCell>{p.floor === null ? '-' : f(p.floor)}</TableCell>
                <TableCell>{p.floor_delta === null ? '-' : signed(p.floor_delta * row.scale, d)}</TableCell>
              </TableRow>
            )
          })}
        </TableBody>
      </Table>
      <details>
        <summary>Report</summary>
        <Typography variant="body2">
          {`Board mount xyz (m) ${result.board_mount.xyz.map((v) => v.toFixed(4)).join(', ')}; rpy (rad) ${result.board_mount.rpy.map((v) => v.toFixed(4)).join(', ')}`}
        </Typography>
        {result.per_view.map((v) => (
          <Typography key={v.view_id} variant="body2" color="text.secondary">
            {`${v.view_id}: arm pose ${v.arm_pose}, RMS ${v.rms_px.toFixed(2)} px${v.held_out ? ' (held out)' : ''}`}
          </Typography>
        ))}
      </details>
    </Box>
  )
}

function Confirm(props: {
  open: boolean
  title: string
  children: string
  confirmLabel: string
  cancelLabel?: string
  onConfirm: () => void
  onCancel: () => void
}) {
  const target = { minHeight: TOUCH_TARGET_PX }
  return (
    <Dialog open={props.open} onClose={props.onCancel}>
      <DialogTitle>{props.title}</DialogTitle>
      <DialogContent>
        <DialogContentText>{props.children}</DialogContentText>
      </DialogContent>
      <DialogActions>
        <Button sx={target} onClick={props.onCancel}>
          {props.cancelLabel ?? 'Cancel'}
        </Button>
        <Button variant="contained" sx={target} onClick={props.onConfirm}>
          {props.confirmLabel}
        </Button>
      </DialogActions>
    </Dialog>
  )
}

/** Head from arm section of the Calibrate page: guided arm sequence, fit against the trusted arm, apply. */
export function HeadArmCalibration() {
  const [sequence, setSequence] = useState<HeadArmSequence | null>(null)
  const [applied, setApplied] = useState<AppliedHeadChain | null>(null)
  const [sessions, setSessions] = useState<HeadArmSession[]>([])
  const [result, setResult] = useState<HeadArmResult | null>(null)
  const [restart, setRestart] = useState<FloorApplied | null>(null)
  const [error, setError] = useState<string | null>(null)
  const [busy, setBusy] = useState(false)
  const [dialog, setDialog] = useState<Dialog>(null)

  const reload = useCallback(async () => {
    const [s, l] = await Promise.all([getHeadArmState(), listHeadArmSessions()])
    if (s.ok) {
      setSequence(s.value.sequence)
      setApplied(s.value.applied)
    }
    if (l.ok) setSessions(l.value)
    const failure = !s.ok ? s : !l.ok ? l : null
    setError(failure ? failure.message : null)
  }, [])

  useEffect(() => {
    void reload()
  }, [reload])

  const running = sequence?.running === true
  useEffect(() => {
    if (!running) return
    const id = setInterval(() => void reload(), ARM_POLL_MS)
    return () => clearInterval(id)
  }, [running, reload])

  /** Runs one sequence call and shows its sequence or its error. */
  const step = async (call: () => Promise<FloorApiResult<HeadArmSequence>>) => {
    setDialog(null)
    const r = await call()
    if (r.ok) {
      setSequence(r.value)
      setError(null)
    } else setError(r.message)
  }

  const sessionId = sequence?.session_id ?? null
  const session = sessions.find((s) => s.id === sessionId) ?? sessions[sessions.length - 1] ?? null
  const canCompute = !busy && !running && session !== null && session.views > 0

  const compute = async () => {
    if (!session) return
    setBusy(true)
    setRestart(null)
    const r = await computeHeadArm(session.id)
    setBusy(false)
    if (r.ok) {
      setResult(r.value)
      setError(null)
    } else {
      setResult(null)
      setError(r.message)
    }
  }

  const apply = async () => {
    if (!result?.passed) return
    setBusy(true)
    const r = await applyHeadArm(result.session, result.id)
    setBusy(false)
    if (r.ok) {
      setRestart(r.value)
      setError(null)
      void reload()
    } else setError(r.message)
  }

  const phase = sequence?.phase ?? PHASE_IDLE
  const skipped = Object.values(sequence?.skipped ?? {}).reduce((a, n) => a + n, 0)
  const target = { minHeight: TOUCH_TARGET_PX }
  const bigJoints = sequence?.big_joints.join(', ') ?? ''
  return (
    <Box component="section" sx={{ mb: 2 }}>
      <Typography variant="h6" component="h2">
        Head from arm
      </Typography>
      <Typography variant="body2" color="text.secondary">
        The head chain is fitted against the trusted arm: the arm holds the ChArUco board in front of the head camera
        and the head pan and tilt are measured against it. Robot not driving.
      </Typography>
      {error && <Alert severity="error">{error}</Alert>}
      {sequence?.error && <Alert severity="error">{sequence.error}</Alert>}

      {phase === PHASE_IDLE && !running && (
        <Box sx={{ mt: 1 }}>
          <Typography variant="body2">
            The arm moves to the calibration pose without the board. Keep the area around the robot clear.
          </Typography>
          <Button variant="contained" sx={{ ...target, mt: 1 }} disabled={busy} onClick={() => setDialog('base')}>
            Move to base pose
          </Button>
          <Button variant="outlined" sx={{ ...target, mt: 1, ml: 1 }} disabled={busy} onClick={() => setDialog('parkBoard')}>
            Park (board is on the gripper)
          </Button>
        </Box>
      )}
      {phase === PHASE_WAITING_BOARD && (
        <Box sx={{ mt: 1 }}>
          <Typography variant="body2">Clip the ChArUco board to the fixed jaw.</Typography>
          <Button variant="contained" sx={{ ...target, mt: 1 }} onClick={() => void step(boardAttached)}>
            Board attached
          </Button>
        </Box>
      )}
      {phase === PHASE_HOLDING && (
        <Box sx={{ mt: 1 }}>
          <Typography variant="body2">
            Press Park to park the arm. The board may stay on the gripper when its park path is clear; otherwise Park is refused and the arm keeps holding.
          </Typography>
          <Button variant="contained" sx={{ ...target, mt: 1 }} onClick={() => setDialog('park')}>
            Park arm
          </Button>
        </Box>
      )}
      {sequence && sequence.session_id && (
        <Box sx={{ mt: 1 }}>
          {running && sequence.total_poses > 0 && (
            <LinearProgress variant="determinate" value={(100 * (sequence.arm_pose ?? 0)) / sequence.total_poses} />
          )}
          <Typography variant="body2">
            {`${sequence.tier === null ? 'Tier -' : `Tier ${sequence.tier}`}, pose ${sequence.arm_pose === null ? '-' : sequence.arm_pose + 1} of ${sequence.total_poses}, ${sequence.captured} captured, ${skipped} skipped - ${sequence.status}`}
          </Typography>
        </Box>
      )}
      <Box sx={{ display: 'flex', gap: 1, mt: 1, flexWrap: 'wrap' }}>
        {(running || phase !== PHASE_IDLE) && (
          <Button variant="outlined" color="error" sx={target} onClick={() => void step(stopSequence)}>
            Stop
          </Button>
        )}
        <Button variant="outlined" sx={target} disabled={!canCompute} onClick={() => void compute()}>
          Compute
        </Button>
      </Box>

      {result && (
        <>
          <ResultView result={result} />
          <Button variant="contained" sx={{ ...target, mt: 1 }} disabled={busy || !result.passed} onClick={() => void apply()}>
            Apply
          </Button>
        </>
      )}
      {restart && (
        <Box sx={{ mt: 1 }}>
          <Typography variant="body2">
            {`Applied: session ${restart.head_chain.session}, result ${restart.head_chain.result}, source ${restart.head_chain.source ?? 'unknown'}. Unit restart state:`}
          </Typography>
          {restart.units.map((u) => (
            <Typography key={u.name} variant="body2">
              {`${u.name}: ${u.restarted ? 'restarted' : 'not restarted'}${u.active_state ? ` (${u.active_state})` : ''}`}
            </Typography>
          ))}
        </Box>
      )}
      {applied && (
        <Typography variant="body2" sx={{ mt: 1 }}>
          {`Applied head chain: session ${applied.session}, result ${applied.result}, source ${applied.source ?? 'unknown'} - ${applied.note}`}
        </Typography>
      )}

      <Confirm
        open={dialog === 'base'}
        title="Move the arm to the base pose"
        confirmLabel="Move"
        onCancel={() => setDialog(null)}
        onConfirm={() => void step(() => moveToBase(true))}
      >
        {`These joints move: ${BASE_JOINTS.join(', ')}. The robot must not be driving and the area around it must be clear.`}
      </Confirm>
      <Confirm
        open={dialog === 'parkBoard'}
        title="The calibration board is on the gripper"
        confirmLabel="Park with the board"
        onCancel={() => setDialog(null)}
        onConfirm={() => void step(() => park(true, true))}
      >
        {`Use this when the arm still holds the board but this page lost the sequence (for example after a restart). The board is recorded as on the gripper, the park path is checked with it, and Park is refused, naming the rule, when the board would hit the floor, the body or the head.${
          sequence?.board_record
            ? ` The node remembers a board clipped in session ${sequence.board_record.session_id ?? 'unknown'} at ${sequence.board_record.clipped_at}: confirm it is still on the gripper.`
            : ''
        }`}
      </Confirm>
      <Confirm
        open={dialog === 'park'}
        title="Park the arm"
        confirmLabel="Park"
        onCancel={() => setDialog(null)}
        onConfirm={() => void step(() => park(true))}
      >
        The arm will move to its park pose, with the board on the gripper or not. Park is refused, naming the rule, when the board would hit the floor, the body or the head.
      </Confirm>
      <Confirm
        open={phase === PHASE_WAITING_BIG}
        title="Allow elbow and shoulder moves"
        confirmLabel="Allow"
        cancelLabel="Skip"
        onCancel={() => void step(() => allowBigJoints(false))}
        onConfirm={() => void step(() => allowBigJoints(true))}
      >
        {`The next poses move ${bigJoints}. Allow them only with the area clear, or skip them.`}
      </Confirm>
    </Box>
  )
}
