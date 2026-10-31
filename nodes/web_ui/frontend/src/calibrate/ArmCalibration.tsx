import { useCallback, useEffect, useState } from 'react'
import Alert from '@mui/material/Alert'
import Box from '@mui/material/Box'
import Button from '@mui/material/Button'
import Chip from '@mui/material/Chip'
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
  ArmResult,
  HeadArmSequence,
  HeadArmSession,
  allowBigJoints,
  applyHeadArm,
  boardAttached,
  computeArm,
  getHeadArmResult,
  getHeadArmState,
  listHeadArmSessions,
  moveToBase,
  stopSequence,
} from './headArmApi'
import { ARM_POLL_MS, Confirm, SkipReasons } from './HeadArmCalibration'
import { downloadUrl } from './rgbdApi'

const M_TO_MM = 1000
const DEG_PER_RAD = 180 / Math.PI
const ARM_KIND = 'arm'
const PHASE_IDLE = 'idle'
const PHASE_WAITING_BOARD = 'waiting_board'
const PHASE_WAITING_BIG = 'waiting_big_joints'
/** Joints of tier 2, named in the confirm when the node has not sent its list. */
const BIG_JOINTS = ['elbow_flex', 'shoulder_lift', 'shoulder_pan']

const signed = (v: number, digits: number) => `${v >= 0 ? '+' : ''}${v.toFixed(digits)}`
const fixed = (v: number | null | undefined, digits: number) => (v === null || v === undefined ? '-' : v.toFixed(digits))

/**
 * Change of one head parameter of an arm result against its URDF start value, scaled and signed.
 *
 * @param result - arm result
 * @param key - head parameter name
 * @param scale - unit scale (M_TO_MM for lengths)
 * @param digits - decimals
 * @returns the signed change, or '-' when the parameter or its URDF value is missing
 */
function headChange(result: ArmResult, key: string, scale: number, digits: number): string {
  const p = result.head?.[key]
  return p && p.urdf !== null ? signed((p.value - p.urdf) * scale, digits) : '-'
}

/**
 * Result card of one arm offsets fit: pass/fail, failed limits, offsets table, tool point, head change, wrist_roll,
 * the snippet with Copy and the zip download.
 *
 * @param result - arm result
 */
function ArmResultCard({ result }: { result: ArmResult }) {
  const tool = result.tool_offset_m?.map((v) => (v * M_TO_MM).toFixed(1)).join(', ')
  const snippet = result.snippet
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
      {result.rms_px !== undefined && (
        <Typography variant="body2" sx={{ mt: 1 }}>
          {`RMS ${fixed(result.rms_px, 2)} px. Held-out RMS ${fixed(result.holdout_rms_px, 2)} px.`}
        </Typography>
      )}
      {result.offsets && (
        <Table size="small" sx={{ mt: 1 }}>
          <TableHead>
            <TableRow>
              {['Joint', 'Current (deg)', 'New (deg)', 'Change (deg)', 'Sigma (deg)'].map((h) => (
                <TableCell key={h}>{h}</TableCell>
              ))}
            </TableRow>
          </TableHead>
          <TableBody>
            {Object.entries(result.offsets).map(([joint, o]) => (
              <TableRow key={joint}>
                <TableCell>{joint}</TableCell>
                <TableCell>{(o.current * DEG_PER_RAD).toFixed(3)}</TableCell>
                <TableCell>{(o.value * DEG_PER_RAD).toFixed(3)}</TableCell>
                <TableCell>{signed((o.value - o.current) * DEG_PER_RAD, 3)}</TableCell>
                <TableCell>{o.sigma === null ? '-' : (o.sigma * DEG_PER_RAD).toFixed(3)}</TableCell>
              </TableRow>
            ))}
          </TableBody>
        </Table>
      )}
      {result.wrist_roll_fitted !== undefined && (
        <Typography variant="body2" sx={{ mt: 1 }}>
          {`wrist_roll ${result.wrist_roll_fitted ? 'fitted' : 'fixed'}`}
        </Typography>
      )}
      {tool !== undefined && (
        <Typography variant="body2">
          {`Tool point ${result.tool_fitted ? `new (mm) ${tool} (fitted)` : `current (mm) ${tool} (kept)`}, held-out error ${fixed(result.holdout_tool_mm, 2)} mm`}
        </Typography>
      )}
      {result.head && (
        <Typography variant="body2">
          {`Head change: x ${headChange(result, 'mount_x_m', M_TO_MM, 2)} mm, y ${headChange(result, 'mount_y_m', M_TO_MM, 2)} mm, yaw ${headChange(result, 'mount_yaw_deg', 1, 3)} deg`}
        </Typography>
      )}
      {snippet !== undefined && (
        <Box sx={{ mt: 1 }}>
          <Box component="pre" sx={{ fontFamily: 'monospace', whiteSpace: 'pre-wrap', m: 0 }}>
            {snippet}
          </Box>
          <Button variant="outlined" sx={{ minHeight: TOUCH_TARGET_PX, mt: 1 }} onClick={() => void navigator.clipboard.writeText(snippet)}>
            Copy
          </Button>
        </Box>
      )}
      <Button
        component="a"
        variant="outlined"
        sx={{ minHeight: TOUCH_TARGET_PX, mt: 1 }}
        href={downloadUrl(result.session, result.id)}
        download
      >
        Download zip
      </Button>
    </Box>
  )
}

/**
 * Arm section of the Calibrate page: the arm joint offsets and tool point fitted from the head D435 watching the
 * board on the fixed jaw. Needs an applied head chain. Drives the head-arm sequence (base pose, board attached:
 * tiers 0 and 1; shoulder moves only after their confirm), computes the arm fit on the session and applies its head
 * part; the offsets snippet is committed by hand.
 */
export function ArmCalibration() {
  const [sequence, setSequence] = useState<HeadArmSequence | null>(null)
  const [applied, setApplied] = useState<AppliedHeadChain | null>(null)
  const [sessions, setSessions] = useState<HeadArmSession[]>([])
  const [result, setResult] = useState<ArmResult | null>(null)
  const [restart, setRestart] = useState<FloorApplied | null>(null)
  const [error, setError] = useState<string | null>(null)
  const [busy, setBusy] = useState(false)
  const [open, setOpen] = useState(false)
  const [confirmShoulder, setConfirmShoulder] = useState(false)
  // Start moved the arm to the base pose: Board attached follows once the sequence waits for the board.
  const [attachPending, setAttachPending] = useState(false)

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
  const step = useCallback(async (call: () => Promise<FloorApiResult<HeadArmSequence>>) => {
    setConfirmShoulder(false)
    const r = await call()
    if (r.ok) {
      setSequence(r.value)
      setError(null)
    } else setError(r.message)
  }, [])

  const phase = sequence?.phase ?? PHASE_IDLE
  useEffect(() => {
    if (!attachPending) return
    if (phase === PHASE_WAITING_BOARD) {
      setAttachPending(false)
      void step(boardAttached)
    } else if (!running) setAttachPending(false)
  }, [attachPending, phase, running, step])

  const sessionId = sequence?.session_id ?? null
  const session = sessions.find((s) => s.id === sessionId) ?? sessions[sessions.length - 1] ?? null
  const latestResult = session?.results[session.results.length - 1] ?? null
  const latestSession = session?.id ?? null
  useEffect(() => {
    if (!latestSession || !latestResult) return
    let live = true
    void getHeadArmResult(latestSession, latestResult).then((r) => {
      if (live && r.ok && r.value.kind === ARM_KIND) setResult(r.value as ArmResult)
    })
    return () => {
      live = false
    }
  }, [latestSession, latestResult])

  const start = async () => {
    const r = await moveToBase(true)
    if (!r.ok) {
      setError(r.message)
      return
    }
    setSequence(r.value)
    setError(null)
    if (r.value.phase === PHASE_WAITING_BOARD) await step(boardAttached)
    else setAttachPending(true)
  }

  const stop = async () => {
    setAttachPending(false)
    await step(stopSequence)
  }

  const compute = async () => {
    if (!session) return
    setBusy(true)
    setRestart(null)
    const r = await computeArm(session.id)
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

  const target = { minHeight: TOUCH_TARGET_PX }
  const canCompute = !busy && !running && session !== null && session.views > 0
  const skipped = Object.values(sequence?.skipped ?? {}).reduce((a, n) => a + n, 0)
  const bigJoints = sequence?.big_joints.length ? sequence.big_joints : BIG_JOINTS
  return (
    <Box component="section" sx={{ mb: 2 }}>
      <Typography variant="h6" component="h2">
        Arm calibration
      </Typography>
      <Typography variant="body2" color="text.secondary">
        The arm joint offsets and tool point are fitted from the head D435 watching the ChArUco board on the fixed jaw,
        through the applied head chain. Robot not driving.
      </Typography>
      <Button variant="contained" sx={{ ...target, mt: 1 }} disabled={applied === null} onClick={() => setOpen(!open)}>
        Arm calibrate
      </Button>
      {applied === null && (
        <Typography variant="body2" color="text.secondary">
          Arm calibrate: apply a head calibration first.
        </Typography>
      )}
      {error && <Alert severity="error">{error}</Alert>}
      {open && applied !== null && (
        <>
          {sequence?.error && <Alert severity="error">{sequence.error}</Alert>}
          <Box sx={{ mt: 1 }}>
            <Typography variant="body2">
              Step 1: board off the floor, clipped to the fixed jaw; the area around the arm clear. Start moves the arm
              to the base pose and runs the slow wrist and elbow poses.
            </Typography>
            <Button variant="contained" sx={{ ...target, mt: 1 }} disabled={busy || running} onClick={() => void start()}>
              Start
            </Button>
          </Box>
          <Box sx={{ mt: 1 }}>
            <Typography variant="body2">Step 2: the shoulder poses move only after their confirm.</Typography>
            <Button
              variant="outlined"
              sx={{ ...target, mt: 1 }}
              disabled={phase !== PHASE_WAITING_BIG}
              onClick={() => setConfirmShoulder(true)}
            >
              Allow shoulder moves
            </Button>
          </Box>
          {sequence && sequence.session_id && (
            <Box sx={{ mt: 1 }}>
              {running && sequence.total_poses > 0 && (
                <LinearProgress variant="determinate" value={(100 * (sequence.arm_pose ?? 0)) / sequence.total_poses} />
              )}
              <Typography variant="body2">
                {`${sequence.tier === null ? 'Tier -' : `Tier ${sequence.tier}`}, step ${sequence.arm_pose === null ? '-' : sequence.arm_pose + 1} of ${sequence.total_poses}, ${sequence.captured} captured, ${skipped} skipped - ${sequence.status}`}
              </Typography>
              <SkipReasons skipped={sequence.skipped} />
            </Box>
          )}
          <Box sx={{ display: 'flex', gap: 1, mt: 1, flexWrap: 'wrap' }}>
            {(running || phase !== PHASE_IDLE) && (
              <Button variant="outlined" color="error" sx={target} onClick={() => void stop()}>
                Stop
              </Button>
            )}
            <Button variant="outlined" sx={target} disabled={!canCompute} onClick={() => void compute()}>
              Compute
            </Button>
          </Box>
          {result && (
            <>
              <ArmResultCard result={result} />
              <Button variant="contained" sx={{ ...target, mt: 1 }} disabled={busy || !result.passed} onClick={() => void apply()}>
                Apply head part
              </Button>
              <Typography variant="body2" color="text.secondary">
                Apply head part applies the fitted head chain only; the offsets are committed by hand from the snippet.
              </Typography>
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
          <Confirm
            open={confirmShoulder}
            title="Allow shoulder moves"
            confirmLabel="Confirm"
            onCancel={() => setConfirmShoulder(false)}
            onConfirm={() => void step(() => allowBigJoints(true))}
          >
            {`The next poses move ${bigJoints.join(', ')}. Allow them only with the area around the arm clear.`}
          </Confirm>
        </>
      )}
    </Box>
  )
}
