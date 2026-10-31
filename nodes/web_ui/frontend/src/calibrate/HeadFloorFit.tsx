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
import {
  AppliedHeadChain,
  FloorApplied,
  FloorParam,
  FloorResult,
  SweepState,
  applyFloor,
  computeFloor,
  getFloorState,
  startSweep,
  stopSweep,
} from './headFloorApi'

/** Sweep state poll period while a sweep runs: 1 Hz. */
export const FLOOR_POLL_MS = 1000

const M_TO_MM = 1000

interface ParamRow {
  label: string
  key: keyof FloorResult['params']
  scale: number
  digits: number
  sigmaDigits: number
}

const PARAM_ROWS: ParamRow[] = [
  { label: 'Mount z (mm)', key: 'mount_z_m', scale: M_TO_MM, digits: 1, sigmaDigits: 2 },
  { label: 'Mount roll (deg)', key: 'mount_roll_deg', scale: 1, digits: 3, sigmaDigits: 3 },
  { label: 'Mount pitch (deg)', key: 'mount_pitch_deg', scale: 1, digits: 3, sigmaDigits: 3 },
  { label: 'Tilt zero (deg)', key: 'tilt_zero_deg', scale: 1, digits: 3, sigmaDigits: 3 },
  { label: 'Tilt roll (deg)', key: 'tilt_roll_deg', scale: 1, digits: 3, sigmaDigits: 3 },
]

const signed = (v: number, digits: number) => `${v >= 0 ? '+' : ''}${v.toFixed(digits)}`

function ParamTableRow({ row, param }: { row: ParamRow; param: FloorParam }) {
  const current = param.urdf * row.scale
  const fitted = param.value * row.scale
  return (
    <TableRow>
      <TableCell>{row.label}</TableCell>
      <TableCell>{current.toFixed(row.digits)}</TableCell>
      <TableCell>{fitted.toFixed(row.digits)}</TableCell>
      <TableCell>{signed(fitted - current, row.digits)}</TableCell>
      <TableCell>{(param.sigma * row.scale).toFixed(row.sigmaDigits)}</TableCell>
    </TableRow>
  )
}

function ResultView({ result }: { result: FloorResult }) {
  return (
    <Box sx={{ mt: 1 }}>
      <Chip label={result.passed ? 'Passed' : 'Failed'} color={result.passed ? 'success' : 'error'} size="small" />
      {result.failed_limits.map((l) => (
        <Alert key={l} severity="error" sx={{ mt: 1 }}>
          {l}
        </Alert>
      ))}
      <Table size="small" sx={{ mt: 1 }}>
        <TableHead>
          <TableRow>
            <TableCell>Parameter</TableCell>
            <TableCell>Current</TableCell>
            <TableCell>Fitted</TableCell>
            <TableCell>Delta</TableCell>
            <TableCell>Sigma</TableCell>
          </TableRow>
        </TableHead>
        <TableBody>
          {PARAM_ROWS.map((row) => {
            const param = result.params[row.key]
            return param ? <ParamTableRow key={row.key} row={row} param={param} /> : null
          })}
        </TableBody>
      </Table>
      <Typography variant="body2" sx={{ mt: 1 }}>
        {`Checked against the limits (pose mean): Height RMS ${result.height_rms_mm} mm, Normal RMS ${result.normal_rms_deg} deg`}
      </Typography>
      {result.observation_height_rms_mm !== undefined && result.observation_normal_rms_deg !== undefined && (
        <Typography variant="body2" color="text.secondary">
          {`Per observation (not limited): Height RMS ${result.observation_height_rms_mm} mm, Normal RMS ${result.observation_normal_rms_deg} deg`}
        </Typography>
      )}
      <Table size="small" sx={{ mt: 1 }}>
        <TableHead>
          <TableRow>
            <TableCell>Pan (deg)</TableCell>
            <TableCell>Tilt (deg)</TableCell>
            <TableCell>Observations</TableCell>
            <TableCell>Height residual (mm)</TableCell>
            <TableCell>Normal residual (deg)</TableCell>
          </TableRow>
        </TableHead>
        <TableBody>
          {result.poses.map((p) => (
            <TableRow key={p.pose}>
              <TableCell>{p.pan_deg.toFixed(1)}</TableCell>
              <TableCell>{p.tilt_deg.toFixed(1)}</TableCell>
              <TableCell>{p.observations}</TableCell>
              <TableCell>{p.height_residual_mm.toFixed(1)}</TableCell>
              <TableCell>{p.normal_residual_deg.toFixed(2)}</TableCell>
            </TableRow>
          ))}
        </TableBody>
      </Table>
      {result.skipped.map((s) => (
        <Typography key={s.pose} variant="body2" color="text.secondary">
          {`Skipped pose ${s.pose} (pan ${s.pan_deg.toFixed(1)}, tilt ${s.tilt_deg.toFixed(1)}): ${s.reason}`}
        </Typography>
      ))}
    </Box>
  )
}

function AppliedView({ applied }: { applied: AppliedHeadChain }) {
  return (
    <Typography variant="body2" sx={{ mt: 1 }}>
      {`Applied head chain: session ${applied.session}, result ${applied.result} - ${applied.note}`}
    </Typography>
  )
}

/** Head floor fit step of the Calibrate page: guided sweep on an empty flat floor, fit, apply. */
export function HeadFloorFit() {
  const [sweep, setSweep] = useState<SweepState | null>(null)
  const [applied, setApplied] = useState<AppliedHeadChain | null>(null)
  const [result, setResult] = useState<FloorResult | null>(null)
  const [restart, setRestart] = useState<FloorApplied | null>(null)
  const [error, setError] = useState<string | null>(null)
  const [busy, setBusy] = useState(false)

  const reload = useCallback(async () => {
    const r = await getFloorState()
    if (r.ok) {
      setSweep(r.value.sweep)
      setApplied(r.value.applied)
      setError(null)
    } else setError(r.message)
  }, [])

  useEffect(() => {
    void reload()
  }, [reload])

  const running = sweep?.running === true
  useEffect(() => {
    if (!running) return
    const id = setInterval(() => void reload(), FLOOR_POLL_MS)
    return () => clearInterval(id)
  }, [running, reload])

  const start = async () => {
    setBusy(true)
    setResult(null)
    setRestart(null)
    const r = await startSweep()
    setBusy(false)
    if (r.ok) {
      setSweep(r.value)
      setError(null)
    } else setError(r.message)
  }

  const stop = async () => {
    const r = await stopSweep()
    if (r.ok) {
      setSweep(r.value)
      setError(null)
    } else setError(r.message)
  }

  const compute = async () => {
    if (!sweep?.session_id) return
    setBusy(true)
    setRestart(null)
    const r = await computeFloor(sweep.session_id)
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
    if (!result) return
    setBusy(true)
    const r = await applyFloor(result.session, result.id)
    setBusy(false)
    if (r.ok) {
      setRestart(r.value)
      setError(null)
      void reload()
    } else setError(r.message)
  }

  const target = { minHeight: TOUCH_TARGET_PX }
  return (
    <Box component="section" sx={{ mb: 2 }}>
      <Typography variant="h6" component="h2">
        Head floor fit
      </Typography>
      <Typography variant="body2" color="text.secondary">
        Robot on a flat floor, not on blocks; about 2 m in front and 1.5 m to each side clear; only the head moves.
        The sweep fits the head mount z, roll, pitch, tilt zero and tilt roll from the floor plane from the D435 depth.
      </Typography>
      {error && <Alert severity="error">{error}</Alert>}
      {sweep?.error && <Alert severity="error">{sweep.error}</Alert>}
      <Box sx={{ display: 'flex', gap: 1, mt: 1, flexWrap: 'wrap' }}>
        <Button variant="contained" sx={target} disabled={busy || running} onClick={() => void start()}>
          Start sweep
        </Button>
        <Button variant="outlined" sx={target} disabled={!running} onClick={() => void stop()}>
          Stop
        </Button>
        <Button
          variant="outlined"
          sx={target}
          disabled={busy || running || !sweep?.session_id}
          onClick={() => void compute()}
        >
          Compute
        </Button>
      </Box>
      {sweep && sweep.session_id && (
        <Box sx={{ mt: 1 }}>
          {running && <LinearProgress variant="determinate" value={sweep.total ? (100 * sweep.pose) / sweep.total : 0} />}
          <Typography variant="body2">
            {`Pose ${sweep.pose} of ${sweep.total}, ${sweep.observations} observations, ${sweep.skipped} skipped - ${sweep.status}`}
          </Typography>
        </Box>
      )}
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
          <Typography variant="body2">Applied. Unit restart state:</Typography>
          {restart.units.map((u) => (
            <Typography key={u.name} variant="body2">
              {`${u.name}: ${u.restarted ? 'restarted' : 'not restarted'}${u.active_state ? ` (${u.active_state})` : ''}`}
            </Typography>
          ))}
        </Box>
      )}
      {applied && <AppliedView applied={applied} />}
    </Box>
  )
}
