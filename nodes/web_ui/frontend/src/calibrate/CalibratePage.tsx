import { useEffect, useState } from 'react'
import Alert from '@mui/material/Alert'
import Box from '@mui/material/Box'
import CircularProgress from '@mui/material/CircularProgress'
import IconButton from '@mui/material/IconButton'
import Typography from '@mui/material/Typography'
import ArrowBackIcon from '@mui/icons-material/ArrowBack'
import { TOUCH_TARGET_PX } from '../theme'
import { clearCalibrateHash } from './calibrateRoute'
import { ImuCalibration } from './ImuCalibration'

/** Body of GET /api/calibration/config: which calibration sections this robot offers. */
export interface CalibrationConfig {
  imu: boolean
}

type LoadState =
  | { kind: 'loading' }
  | { kind: 'not_configured' }
  | { kind: 'error'; message: string }
  | { kind: 'ready'; config: CalibrationConfig }

async function loadConfig(): Promise<LoadState> {
  const res = await fetch('/api/calibration/config')
  if (res.status === 404) return { kind: 'not_configured' }
  if (!res.ok) return { kind: 'error', message: `HTTP ${res.status}` }
  const body = (await res.json()) as Partial<CalibrationConfig>
  return { kind: 'ready', config: { imu: body.imu === true } }
}

interface Props {
  /** Called after the hash is cleared, to return to the stored tab. */
  onBack: () => void
}

/** Hidden Calibrate page, reached from the hamburger menu at #/calibrate. */
export function CalibratePage({ onBack }: Props) {
  const [state, setState] = useState<LoadState>({ kind: 'loading' })

  useEffect(() => {
    let live = true
    loadConfig()
      .then((s) => live && setState(s))
      .catch((e: unknown) => live && setState({ kind: 'error', message: e instanceof Error ? e.message : String(e) }))
    return () => {
      live = false
    }
  }, [])

  const back = () => {
    clearCalibrateHash()
    onBack()
  }

  return (
    <Box sx={{ height: '100%', overflow: 'auto', p: 2 }}>
      <Box sx={{ display: 'flex', alignItems: 'center', gap: 1, mb: 2 }}>
        <IconButton aria-label="Back" onClick={back} edge="start" sx={{ minWidth: TOUCH_TARGET_PX, minHeight: TOUCH_TARGET_PX }}>
          <ArrowBackIcon />
        </IconButton>
        <Typography variant="h5" component="h1">
          Calibrate
        </Typography>
      </Box>
      {state.kind === 'loading' && <CircularProgress size={24} />}
      {state.kind === 'not_configured' && <Alert severity="info">Calibration page not configured on this robot</Alert>}
      {state.kind === 'error' && <Alert severity="error">Calibration config failed to load: {state.message}</Alert>}
      {state.kind === 'ready' && (
        <>
          {state.config.imu && <ImuCalibration />}
          <Typography color="text.secondary">Arm calibration - deferred</Typography>
        </>
      )}
    </Box>
  )
}
