/** Head floor fit parts of the Camera to ground flow: the sweep / compute / apply state and the result details. */
import { useCallback, useEffect, useState } from 'react'
import Box from '@mui/material/Box'
import Table from '@mui/material/Table'
import TableBody from '@mui/material/TableBody'
import TableCell from '@mui/material/TableCell'
import TableHead from '@mui/material/TableHead'
import TableRow from '@mui/material/TableRow'
import Typography from '@mui/material/Typography'
import {
  AppliedHeadChain,
  FloorApplied,
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

/** State and actions of the head floor fit (one sweep, its compute and the apply of a passed result). */
export interface FloorFit {
  sweep: SweepState | null
  applied: AppliedHeadChain | null
  result: FloorResult | null
  restart: FloorApplied | null
  error: string | null
  busy: boolean
  running: boolean
  start: () => Promise<void>
  stop: () => Promise<void>
  compute: () => Promise<void>
  /** Applies the shown result; resolves true when it was applied. */
  apply: () => Promise<boolean>
}

/**
 * The head floor fit state: polls the node at FLOOR_POLL_MS while a sweep runs.
 *
 * @returns the state and its actions
 */
export function useFloorFit(): FloorFit {
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
    if (!result?.passed) return false
    setBusy(true)
    const r = await applyFloor(result.session, result.id)
    setBusy(false)
    if (!r.ok) {
      setError(r.message)
      return false
    }
    setRestart(r.value)
    setError(null)
    void reload()
    return true
  }

  return { sweep, applied, result, restart, error, busy, running, start, stop, compute, apply }
}

/**
 * Details of a floor fit next to its gate: the limit RMS values, the per-pose residuals and the skipped poses.
 *
 * @param result - floor fit result
 */
export function FloorResultDetails({ result }: { result: FloorResult }) {
  return (
    <Box sx={{ mt: 1 }}>
      <Typography variant="body2">
        {`Checked against the limits (pose mean): Height RMS ${result.height_rms_mm} mm, Normal RMS ${result.normal_rms_deg} deg`}
      </Typography>
      {result.observation_height_rms_mm !== undefined && result.observation_normal_rms_deg !== undefined && (
        <Typography variant="body2" color="text.secondary">
          {`Per observation (not limited): Height RMS ${result.observation_height_rms_mm} mm, Normal RMS ${result.observation_normal_rms_deg} deg`}
        </Typography>
      )}
      <Box sx={{ overflowX: 'auto' }}>
        <Table size="small" sx={{ mt: 1 }} aria-label="Residuals per pose">
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
      </Box>
      {result.skipped.map((s) => (
        <Typography key={s.pose} variant="body2" color="text.secondary">
          {`Skipped pose ${s.pose} (pan ${s.pan_deg.toFixed(1)}, tilt ${s.tilt_deg.toFixed(1)}): ${s.reason}`}
        </Typography>
      ))}
    </Box>
  )
}

/**
 * Unit restart state after an apply.
 *
 * @param restart - the apply answer
 */
export function RestartView({ restart }: { restart: FloorApplied }) {
  return (
    <Box sx={{ mt: 1 }}>
      <Typography variant="body2">Applied. Unit restart state:</Typography>
      {restart.units.map((u) => (
        <Typography key={u.name} variant="body2">
          {`${u.name}: ${u.restarted ? 'restarted' : 'not restarted'}${u.active_state ? ` (${u.active_state})` : ''}`}
        </Typography>
      ))}
    </Box>
  )
}

/**
 * The applied head chain record.
 *
 * @param applied - the applied record
 */
export function AppliedView({ applied }: { applied: AppliedHeadChain }) {
  return (
    <Typography variant="body2" sx={{ mt: 1 }}>
      {`Applied head chain: session ${applied.session}, result ${applied.result}, source ${applied.source ?? 'unknown'} - ${applied.note}`}
    </Typography>
  )
}
