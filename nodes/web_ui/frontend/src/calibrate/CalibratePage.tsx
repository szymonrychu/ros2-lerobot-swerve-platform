import { useEffect, useState } from 'react'
import Alert from '@mui/material/Alert'
import Box from '@mui/material/Box'
import CircularProgress from '@mui/material/CircularProgress'
import IconButton from '@mui/material/IconButton'
import Tab from '@mui/material/Tab'
import Tabs from '@mui/material/Tabs'
import Typography from '@mui/material/Typography'
import ArrowBackIcon from '@mui/icons-material/ArrowBack'
import { TOUCH_TARGET_PX } from '../theme'
import { FLOW_IDS, FlowId, calibrateFlow, clearCalibrateHash, flowHash } from './calibrateRoute'
import { GENERAL_ACTIONS, GeneralActionId } from './generalApi'
import { GeneralSection } from './GeneralSection'
import type { HeadCameraPreview } from './GroundBoard'
import { ArmCameraFlow } from './flows/ArmCameraFlow'
import { CameraGroundFlow } from './flows/CameraGroundFlow'
import { ImuFlow } from './flows/ImuFlow'
import { ServoHomeFlow } from './flows/ServoHomeFlow'

/** Body of GET /api/calibration/config: which calibration flows and General actions this robot offers. */
export interface CalibrationConfig {
  imu: boolean
  rgbd: boolean
  servos: boolean
  /** Offered General actions, page order. */
  general: GeneralActionId[]
}

/** Tab label and section title of each flow. */
const FLOWS: Record<FlowId, { tab: string; title: string }> = {
  'servo-home': { tab: 'Servo home', title: 'Servo home calibration' },
  imu: { tab: 'IMU', title: 'IMU calibration' },
  'camera-ground': { tab: 'Camera to ground', title: 'Camera to ground calibration' },
  'arm-camera': { tab: 'Arm to camera', title: 'Arm to camera calibration' },
}
const GENERAL_IDS: string[] = GENERAL_ACTIONS.map((a) => a.id)

type LoadState =
  | { kind: 'loading' }
  | { kind: 'not_configured' }
  | { kind: 'error'; message: string }
  | { kind: 'ready'; config: CalibrationConfig }

/**
 * Parse the config body; unknown General actions are dropped.
 *
 * @param body - JSON body of GET /api/calibration/config
 * @returns the config
 */
export function parseConfig(body: Partial<Record<keyof CalibrationConfig, unknown>>): CalibrationConfig {
  const general = Array.isArray(body.general) ? body.general.filter((a): a is GeneralActionId => GENERAL_IDS.includes(a as string)) : []
  return { imu: body.imu === true, rgbd: body.rgbd === true, servos: body.servos === true, general }
}

async function loadConfig(): Promise<LoadState> {
  const res = await fetch('/api/calibration/config')
  if (res.status === 404) return { kind: 'not_configured' }
  if (!res.ok) return { kind: 'error', message: `HTTP ${res.status}` }
  return { kind: 'ready', config: parseConfig((await res.json()) as Partial<Record<keyof CalibrationConfig, unknown>>) }
}

/**
 * The flows a config offers, in page order.
 *
 * @param config - calibration config
 * @returns offered flow ids
 */
export function offeredFlows(config: CalibrationConfig): FlowId[] {
  const offered: Record<FlowId, boolean> = {
    'servo-home': config.servos,
    imu: config.imu,
    'camera-ground': config.rgbd,
    'arm-camera': config.rgbd,
  }
  return FLOW_IDS.filter((f) => offered[f])
}

interface Props {
  /** Called after the hash is cleared, to return to the stored tab. */
  onBack: () => void
  /** The head camera stream for the Camera to ground board check, null when the robot config has none. */
  headCamera?: HeadCameraPreview | null
}

/**
 * Hidden Calibrate page, reached from the hamburger menu at #/calibrate: the General section on top, then one tab per
 * calibration flow (one open at a time, deep link #/calibrate/<flow>).
 */
export function CalibratePage({ onBack, headCamera = null }: Props) {
  const [state, setState] = useState<LoadState>({ kind: 'loading' })
  const [hashFlow, setHashFlow] = useState<FlowId | null>(() => calibrateFlow(window.location.hash))

  useEffect(() => {
    let live = true
    loadConfig()
      .then((s) => live && setState(s))
      .catch((e: unknown) => live && setState({ kind: 'error', message: e instanceof Error ? e.message : String(e) }))
    return () => {
      live = false
    }
  }, [])

  useEffect(() => {
    const sync = () => setHashFlow(calibrateFlow(window.location.hash))
    window.addEventListener('hashchange', sync)
    return () => window.removeEventListener('hashchange', sync)
  }, [])

  const back = () => {
    clearCalibrateHash()
    onBack()
  }

  const select = (flow: FlowId) => {
    window.history.replaceState(null, '', flowHash(flow))
    setHashFlow(flow)
  }

  const flows = state.kind === 'ready' ? offeredFlows(state.config) : []
  const flow = hashFlow !== null && flows.includes(hashFlow) ? hashFlow : flows[0] ?? null
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
          {(state.config.rgbd || state.config.general.length > 0) && (
            <GeneralSection actions={state.config.general} rgbd={state.config.rgbd} />
          )}
          {flow !== null && (
            <>
              <Tabs
                value={flow}
                onChange={(_, next: FlowId) => select(next)}
                variant="scrollable"
                scrollButtons="auto"
                aria-label="Calibration flows"
                sx={{ borderBottom: 1, borderColor: 'divider' }}
              >
                {flows.map((f) => (
                  <Tab key={f} value={f} label={FLOWS[f].tab} id={`calibrate-tab-${f}`} aria-controls={`calibrate-panel-${f}`} sx={{ minHeight: TOUCH_TARGET_PX }} />
                ))}
              </Tabs>
              <Box component="section" role="tabpanel" id={`calibrate-panel-${flow}`} aria-labelledby={`calibrate-tab-${flow}`} sx={{ pt: 2 }}>
                <Typography variant="h6" component="h2" sx={{ mb: 1 }}>
                  {FLOWS[flow].title}
                </Typography>
                {flow === 'servo-home' && <ServoHomeFlow />}
                {flow === 'imu' && <ImuFlow />}
                {flow === 'camera-ground' && (
                  <CameraGroundFlow canHomeHead={state.config.general.includes('home_head')} headCamera={headCamera} />
                )}
                {flow === 'arm-camera' && <ArmCameraFlow canHomeArm={state.config.general.includes('home_arm')} />}
              </Box>
            </>
          )}
        </>
      )}
    </Box>
  )
}
