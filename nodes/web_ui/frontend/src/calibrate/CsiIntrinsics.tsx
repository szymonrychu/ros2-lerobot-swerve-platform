import { useCallback, useEffect, useState } from 'react'
import Alert from '@mui/material/Alert'
import Box from '@mui/material/Box'
import Button from '@mui/material/Button'
import Table from '@mui/material/Table'
import TableBody from '@mui/material/TableBody'
import TableCell from '@mui/material/TableCell'
import TableRow from '@mui/material/TableRow'
import ToggleButton from '@mui/material/ToggleButton'
import ToggleButtonGroup from '@mui/material/ToggleButtonGroup'
import Typography from '@mui/material/Typography'
import { TOUCH_TARGET_PX } from '../theme'
import { CoverageGrid } from './CoverageGrid'
import {
  CSI_CAMERAS,
  type CsiApplyAnswer,
  type CsiCamera,
  type CsiResult,
  type CsiState,
  applyCsi,
  computeCsi,
  getCsiState,
  startCsiCapture,
  stopCsiCapture,
} from './csiApi'

/** State poll period while a capture runs: 1 Hz. */
export const CSI_POLL_MS = 1000

const LABELS: Record<CsiCamera, string> = { front: 'Front', back: 'Back' }

/** (label, row, column) of the K entries shown. */
const K_ROWS: [string, number, number][] = [
  ['fx', 0, 0],
  ['fy', 1, 1],
  ['cx', 0, 2],
  ['cy', 1, 2],
]

function ResultView({ result }: { result: CsiResult }) {
  return (
    <Box sx={{ mt: 1 }}>
      <Typography variant="body2">{`RMS ${result.rms.toFixed(3)} px over ${result.view_count} views`}</Typography>
      <Table size="small" sx={{ mt: 1, maxWidth: 360 }}>
        <TableBody>
          {K_ROWS.map(([label, r, c]) => (
            <TableRow key={label}>
              <TableCell>{label}</TableCell>
              <TableCell>{result.k[r][c].toFixed(1)}</TableCell>
            </TableRow>
          ))}
          <TableRow>
            <TableCell>D (plumb_bob)</TableCell>
            <TableCell>{result.d.map((v) => v.toFixed(4)).join(', ')}</TableCell>
          </TableRow>
        </TableBody>
      </Table>
    </Box>
  )
}

/** CSI camera intrinsics section of the Calibrate page: ChArUco views of the front or back camera, compute, apply. */
export function CsiIntrinsics() {
  const [camera, setCamera] = useState<CsiCamera>(CSI_CAMERAS[0])
  const [state, setState] = useState<CsiState | null>(null)
  const [result, setResult] = useState<CsiResult | null>(null)
  const [restart, setRestart] = useState<CsiApplyAnswer | null>(null)
  const [error, setError] = useState<string | null>(null)
  const [busy, setBusy] = useState(false)

  const reload = useCallback(async () => {
    const r = await getCsiState(camera)
    if (r.ok) {
      setState(r.value)
      setError(null)
    } else setError(r.message)
  }, [camera])

  useEffect(() => {
    void reload()
  }, [reload])

  const running = state?.capture.running === true
  useEffect(() => {
    if (!running) return
    const id = setInterval(() => void reload(), CSI_POLL_MS)
    return () => clearInterval(id)
  }, [running, reload])

  const select = (next: CsiCamera | null) => {
    if (next === null || next === camera) return
    setCamera(next)
    setState(null)
    setResult(null)
    setRestart(null)
  }

  const answer = (r: Awaited<ReturnType<typeof startCsiCapture>>) => {
    if (r.ok) {
      setState(r.value)
      setError(null)
    } else setError(r.message)
  }

  const start = async () => {
    setBusy(true)
    setResult(null)
    setRestart(null)
    answer(await startCsiCapture(camera))
    setBusy(false)
  }

  const stop = async () => answer(await stopCsiCapture(camera))

  const session = state?.session ?? null
  const compute = async () => {
    if (!session) return
    setBusy(true)
    setRestart(null)
    const r = await computeCsi(camera, session.id)
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
    const r = await applyCsi(camera, result.session, result.result)
    setBusy(false)
    if (r.ok) {
      setRestart(r.value)
      setError(null)
      void reload()
    } else setError(r.message)
  }

  const target = { minHeight: TOUCH_TARGET_PX }
  const capture = state?.capture
  const ownCapture = capture !== undefined && capture.camera === camera && capture.session_id !== null
  return (
    <Box component="section" sx={{ mb: 2 }}>
      <Typography variant="h6" component="h2">
        CSI camera intrinsics
      </Typography>
      <Typography variant="body2" color="text.secondary">
        Hold the ChArUco board in front of the camera and move it between views: near and far, tilted, into every
        corner of the image. A view is kept when the board is found and it moved from the last kept view.
      </Typography>
      <ToggleButtonGroup
        exclusive
        size="small"
        value={camera}
        onChange={(_, next: CsiCamera | null) => select(next)}
        aria-label="CSI camera"
        sx={{ mt: 1 }}
      >
        {CSI_CAMERAS.map((name) => (
          <ToggleButton key={name} value={name} sx={target}>
            {LABELS[name]}
          </ToggleButton>
        ))}
      </ToggleButtonGroup>
      {error && <Alert severity="error">{error}</Alert>}
      <Box sx={{ display: 'flex', gap: 1, mt: 1, flexWrap: 'wrap' }}>
        <Button variant="contained" sx={target} disabled={busy || running} onClick={() => void start()}>
          Start capture
        </Button>
        <Button variant="outlined" sx={target} disabled={!running || capture?.camera !== camera} onClick={() => void stop()}>
          Stop
        </Button>
        <Button variant="outlined" sx={target} disabled={busy || running || !session} onClick={() => void compute()}>
          Compute
        </Button>
      </Box>
      {ownCapture && capture && (
        <Typography variant="body2" sx={{ mt: 1 }}>{`${capture.view_count} views kept - ${capture.status}`}</Typography>
      )}
      {running && capture?.camera !== camera && (
        <Typography variant="body2" sx={{ mt: 1 }}>{`A ${capture?.camera} capture is running`}</Typography>
      )}
      {session && (
        <Box sx={{ mt: 1 }}>
          <Typography variant="body2">{`Session ${session.id}: ${session.view_count} views`}</Typography>
          <CoverageGrid rgb={session.coverage} />
        </Box>
      )}
      {result && (
        <>
          <ResultView result={result} />
          <Button variant="contained" sx={{ ...target, mt: 1 }} disabled={busy} onClick={() => void apply()}>
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
      {state?.applied && (
        <Typography variant="body2" sx={{ mt: 1 }}>
          {`Applied: session ${state.applied.session}, result ${state.applied.result} (${state.applied.applied_at})`}
        </Typography>
      )}
    </Box>
  )
}
