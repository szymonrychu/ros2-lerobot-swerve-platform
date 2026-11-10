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
import { BoardNeeded } from './BoardNeeded'
import { CoverageGrid } from './CoverageGrid'
import {
  type CsiApplyAnswer,
  type CsiResult,
  type CsiState,
  applyCsi,
  computeCsi,
  getCsiCameras,
  getCsiState,
  startCsiCapture,
  stopCsiCapture,
} from './csiApi'

/** State poll period while a capture runs: 1 Hz. */
export const CSI_POLL_MS = 1000

/**
 * Display label of a camera name: "front" -> "Front".
 *
 * @param name - the node's camera name
 * @returns the label
 */
export function cameraLabel(name: string): string {
  return name.charAt(0).toUpperCase() + name.slice(1)
}

/**
 * The camera-not-detected message of a capture that got no frame (capture.no_frames).
 *
 * @param camera - the camera name
 * @returns the message shown in place of an endless "waiting for frames"
 */
export function noFramesText(camera: string): string {
  return `Camera not detected: the ${cameraLabel(camera)} camera published no frames. Check the ribbon cable and reboot the robot, then start the capture again.`
}

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

/**
 * CSI camera intrinsics section of the Calibrate page: ChArUco views of an enabled CSI camera (GET /csi, so a disabled
 * camera node is not offered), compute, apply. A capture that gets no frame says the camera is not detected.
 */
export function CsiIntrinsics({ heading = true }: { heading?: boolean } = {}) {
  // Enabled cameras from the node; null until loaded.
  const [cameras, setCameras] = useState<string[] | null>(null)
  const [camera, setCamera] = useState<string | null>(null)
  const [state, setState] = useState<CsiState | null>(null)
  const [result, setResult] = useState<CsiResult | null>(null)
  const [restart, setRestart] = useState<CsiApplyAnswer | null>(null)
  const [error, setError] = useState<string | null>(null)
  const [busy, setBusy] = useState(false)

  useEffect(() => {
    let live = true
    void getCsiCameras().then((r) => {
      if (!live) return
      if (r.ok) {
        setCameras(r.value.cameras)
        setCamera(r.value.cameras[0] ?? null)
      } else setError(r.message)
    })
    return () => {
      live = false
    }
  }, [])

  const reload = useCallback(async () => {
    if (camera === null) return
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

  const select = (next: string | null) => {
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
    if (camera === null) return
    setBusy(true)
    setResult(null)
    setRestart(null)
    answer(await startCsiCapture(camera))
    setBusy(false)
  }

  const stop = async () => {
    if (camera !== null) answer(await stopCsiCapture(camera))
  }

  const session = state?.session ?? null
  const compute = async () => {
    if (!session || camera === null) return
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
    if (!result || camera === null) return
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
      {heading && (
        <Typography variant="h6" component="h2">
          CSI camera intrinsics
        </Typography>
      )}
      <Typography variant="body2" color="text.secondary">
        Hold the ChArUco board in front of the camera and move it between views: near and far, tilted, into every
        corner of the image. A view is kept when the board is found and it moved from the last kept view.
      </Typography>
      <BoardNeeded role="full" />
      {cameras !== null && cameras.length > 1 && (
        <ToggleButtonGroup
          exclusive
          size="small"
          value={camera}
          onChange={(_, next: string | null) => select(next)}
          aria-label="CSI camera"
          sx={{ mt: 1 }}
        >
          {cameras.map((name) => (
            <ToggleButton key={name} value={name} sx={target}>
              {cameraLabel(name)}
            </ToggleButton>
          ))}
        </ToggleButtonGroup>
      )}
      {cameras !== null && cameras.length === 1 && camera !== null && (
        <Typography variant="body2" sx={{ mt: 1 }}>{`Camera: ${cameraLabel(camera)}`}</Typography>
      )}
      {cameras !== null && cameras.length === 0 && (
        <Alert severity="info" sx={{ mt: 1 }}>No CSI camera is enabled on this robot.</Alert>
      )}
      {error && <Alert severity="error">{error}</Alert>}
      {ownCapture && capture?.no_frames && camera !== null && (
        <Alert severity="error" sx={{ mt: 1 }}>
          {noFramesText(camera)}
        </Alert>
      )}
      <Box sx={{ display: 'flex', gap: 1, mt: 1, flexWrap: 'wrap' }}>
        <Button variant="contained" sx={target} disabled={busy || running || camera === null} onClick={() => void start()}>
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
