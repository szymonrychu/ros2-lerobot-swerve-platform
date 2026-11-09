/**
 * More fits on a head-arm capture session, reachable from the Arm to camera flow: the head-from-arm fit (head chain
 * against the trusted arm) and the arm offsets fit (joint offsets snippet committed by hand, head part applied).
 */
import { useEffect, useState } from 'react'
import Alert from '@mui/material/Alert'
import Box from '@mui/material/Box'
import Button from '@mui/material/Button'
import Chip from '@mui/material/Chip'
import Table from '@mui/material/Table'
import TableBody from '@mui/material/TableBody'
import TableCell from '@mui/material/TableCell'
import TableHead from '@mui/material/TableHead'
import TableRow from '@mui/material/TableRow'
import Typography from '@mui/material/Typography'
import { TOUCH_TARGET_PX } from '../theme'
import type { FloorApplied } from './headFloorApi'
import {
  ArmResult,
  HeadArmResult,
  HeadArmSession,
  applyHeadArm,
  computeArm,
  computeHeadArm,
  getHeadArmResult,
} from './headArmApi'
import { downloadUrl } from './rgbdApi'

const M_TO_MM = 1000
const DEG_PER_RAD = 180 / Math.PI
const ARM_KIND = 'arm'

const signed = (v: number, digits: number) => `${v >= 0 ? '+' : ''}${v.toFixed(digits)}`
const mm = (m: number[] | undefined) => m?.map((v) => (v * M_TO_MM).toFixed(1)).join(', ')
const fixed = (v: number | null | undefined, digits: number) => (v === null || v === undefined ? '-' : v.toFixed(digits))

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
  // d435_joint (head_tilt -> D435): x, z, roll, yaw are fitted; y and pitch stay at the URDF values (design D32b).
  { label: 'D435 x (mm)', key: 'camera_x_m', scale: M_TO_MM, digits: 2 },
  { label: 'D435 z (mm)', key: 'camera_z_m', scale: M_TO_MM, digits: 2 },
  { label: 'D435 roll (deg)', key: 'camera_roll_deg', scale: 1, digits: 3 },
  { label: 'D435 yaw (deg)', key: 'camera_yaw_deg', scale: 1, digits: 3 },
  // Head pan axis position relative to the head (head_pan origin x, y); only when the fit frees it.
  { label: 'Pan axis x (mm)', key: 'pan_axis_x_m', scale: M_TO_MM, digits: 2 },
  { label: 'Pan axis y (mm)', key: 'pan_axis_y_m', scale: M_TO_MM, digits: 2 },
]

function HeadArmResultView({ result }: { result: HeadArmResult }) {
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
      {result.offset_rms_px?.map((o) => (
        <Typography key={o.head_offset} variant="body2" color="text.secondary">
          {`Head offset ${o.head_offset}: ${o.views} views, RMS ${o.rms_px.toFixed(2)} px`}
        </Typography>
      ))}
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
  const tool = mm(result.tool_offset_m)
  const current = mm(result.tool_offset_current_m)
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
          {`Tool point ${result.tool_fitted ? `current (mm) ${current ?? '-'}, new (mm) ${tool} (fitted)` : `current (mm) ${tool} (kept)`}, held-out error ${fixed(result.holdout_tool_mm, 2)} mm`}
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
 * Applied head chain and the unit restart state after an apply.
 *
 * @param restart - the apply answer
 */
function AppliedRestart({ restart }: { restart: FloorApplied }) {
  return (
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
  )
}

interface Props {
  /** The capture session the fits run on; null before the first capture. */
  session: HeadArmSession | null
  /** True while a sequence runs (no compute then). */
  disabled: boolean
}

/**
 * The head-from-arm and arm offsets fits of one session, behind a "More fits on this session" toggle.
 *
 * @param props - see Props
 */
export function MoreFits({ session, disabled }: Props) {
  const [open, setOpen] = useState(false)
  const [headResult, setHeadResult] = useState<HeadArmResult | null>(null)
  const [armResult, setArmResult] = useState<ArmResult | null>(null)
  const [restart, setRestart] = useState<FloorApplied | null>(null)
  const [error, setError] = useState<string | null>(null)
  const [busy, setBusy] = useState(false)

  const sessionId = session?.id ?? null
  const latest = session?.results[session.results.length - 1] ?? null
  useEffect(() => {
    if (!open || !sessionId || !latest) return
    let live = true
    void getHeadArmResult(sessionId, latest).then((r) => {
      if (live && r.ok && r.value.kind === ARM_KIND) setArmResult(r.value as ArmResult)
    })
    return () => {
      live = false
    }
  }, [open, sessionId, latest])

  const run = async <T,>(call: () => Promise<{ ok: true; value: T } | { ok: false; message: string }>, done: (v: T) => void) => {
    setBusy(true)
    const r = await call()
    setBusy(false)
    if (r.ok) {
      done(r.value)
      setError(null)
    } else setError(r.message)
  }

  const target = { minHeight: TOUCH_TARGET_PX }
  const canCompute = !busy && !disabled && session !== null && session.views > 0
  return (
    <Box sx={{ mt: 1 }}>
      <Button variant="outlined" sx={target} onClick={() => setOpen(!open)}>
        More fits on this session
      </Button>
      {open && (
        <Box sx={{ mt: 1 }}>
          {session === null && <Typography variant="body2">No capture session yet.</Typography>}
          {error && <Alert severity="error">{error}</Alert>}
          <Typography variant="subtitle2" component="h4" sx={{ mt: 1 }}>
            Head from arm
          </Typography>
          <Typography variant="body2" color="text.secondary">
            Fits the head chain against the trusted arm from the same views: the head D435 watches the ARM board held by its
            handle in the jaws while the arm moves it.
          </Typography>
          <Box sx={{ display: 'flex', gap: 1, mt: 1, flexWrap: 'wrap' }}>
            <Button
              variant="outlined"
              sx={target}
              disabled={!canCompute}
              onClick={() => session && void run(() => computeHeadArm(session.id), (v) => {
                    setRestart(null)
                    setHeadResult(v)
                  })}
            >
              Compute head from arm
            </Button>
          </Box>
          {headResult && (
            <>
              <HeadArmResultView result={headResult} />
              <Button
                variant="contained"
                sx={{ ...target, mt: 1 }}
                disabled={busy || !headResult.passed}
                onClick={() => void run(() => applyHeadArm(headResult.session, headResult.id), setRestart)}
              >
                Apply head from arm
              </Button>
            </>
          )}
          <Typography variant="subtitle2" component="h4" sx={{ mt: 2 }}>
            Arm offsets
          </Typography>
          <Typography variant="body2" color="text.secondary">
            Fits the arm joint offsets and tool point through the applied head chain; the offsets snippet is committed by hand.
          </Typography>
          <Box sx={{ display: 'flex', gap: 1, mt: 1, flexWrap: 'wrap' }}>
            <Button
              variant="outlined"
              sx={target}
              disabled={!canCompute}
              onClick={() => session && void run(() => computeArm(session.id), (v) => {
                    setRestart(null)
                    setArmResult(v)
                  })}
            >
              Compute arm offsets
            </Button>
          </Box>
          {armResult && (
            <>
              <ArmResultCard result={armResult} />
              <Button
                variant="contained"
                sx={{ ...target, mt: 1 }}
                disabled={busy || !armResult.passed}
                onClick={() => void run(() => applyHeadArm(armResult.session, armResult.id), setRestart)}
              >
                Apply head part
              </Button>
              <Typography variant="body2" color="text.secondary">
                Apply head part applies the fitted head chain only; the offsets are committed by hand from the snippet.
              </Typography>
            </>
          )}
          {restart && <AppliedRestart restart={restart} />}
        </Box>
      )}
    </Box>
  )
}
