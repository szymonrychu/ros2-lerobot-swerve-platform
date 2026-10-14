import { useCallback, useEffect, useState } from 'react'
import Alert from '@mui/material/Alert'
import Box from '@mui/material/Box'
import Button from '@mui/material/Button'
import Typography from '@mui/material/Typography'
import { ConfirmDialog } from '../layout/ConfirmDialog'
import { TOUCH_TARGET_PX } from '../theme'
import {
  IMU_LEVEL_MAX,
  ImuStatus,
  fetchImuStatus,
  imuLevelColor,
  postImuReset,
  postImuSave,
  saveEnabled,
} from './imuApi'

/** Status poll period: 1 Hz. */
export const IMU_POLL_MS = 1000
const RESET_TEXT = 'The IMU forgets its calibration and starts from zero.'

const LEVELS = ['sys', 'gyro', 'accel', 'mag'] as const
const GUIDANCE: Record<string, string> = {
  gyro: 'gyro: keep the robot still for a few seconds.',
  accel: 'accel: hold a few static poses, lifting the robot and setting it down on different sides.',
  mag: 'mag: rotate the robot or drive figure-eights, away from motors and metal.',
}

type StatusState = { kind: 'loading' } | { kind: 'ok'; status: ImuStatus } | { kind: 'error'; message: string }
type Feedback = { ok: boolean; message: string } | null

/** IMU calibration section of the Calibrate page. Polls only after Calibrate IMU is pressed, while the tab is visible. */
export function ImuCalibration() {
  const [active, setActive] = useState(false)
  const [state, setState] = useState<StatusState>({ kind: 'loading' })
  const [feedback, setFeedback] = useState<Feedback>(null)
  const [confirmOpen, setConfirmOpen] = useState(false)

  useEffect(() => {
    if (!active) return
    let live = true
    const poll = async () => {
      if (document.visibilityState === 'hidden') return
      const r = await fetchImuStatus()
      if (!live) return
      setState(r.ok ? { kind: 'ok', status: r.value } : { kind: 'error', message: r.message })
    }
    void poll()
    const id = setInterval(() => void poll(), IMU_POLL_MS)
    return () => {
      live = false
      clearInterval(id)
    }
  }, [active])

  const act = useCallback(async (call: typeof postImuSave) => {
    const r = await call()
    setFeedback(r.ok ? { ok: r.value.ok, message: r.value.message } : { ok: false, message: r.message })
  }, [])

  const status = state.kind === 'ok' ? state.status : null
  const btn = { minHeight: TOUCH_TARGET_PX }

  return (
    <Box component="section" sx={{ mb: 2 }}>
      <Typography variant="h6" component="h2">
        IMU calibration
      </Typography>
      <Box sx={{ display: 'flex', gap: 1, flexWrap: 'wrap', my: 1 }}>
        <Button variant="contained" sx={btn} onClick={() => setActive(true)}>
          Calibrate IMU
        </Button>
        <Button variant="outlined" sx={btn} disabled={!saveEnabled(status)} onClick={() => void act(postImuSave)}>
          Save offsets
        </Button>
        <Button variant="outlined" color="error" sx={btn} onClick={() => setConfirmOpen(true)}>
          Reset IMU calibration
        </Button>
      </Box>
      {active && state.kind === 'error' && <Alert severity="warning">{state.message}</Alert>}
      {active && status && (
        <Box>
          {LEVELS.map((k) => (
            <Typography key={k} sx={{ color: imuLevelColor(status[k]) }}>
              {`${k} ${status[k]} / ${IMU_LEVEL_MAX}`}
            </Typography>
          ))}
          <Typography>{`Restored from saved offsets: ${status.restored ? 'yes' : 'no'}`}</Typography>
        </Box>
      )}
      {active && (
        <Box sx={{ mt: 1 }}>
          {Object.values(GUIDANCE).map((g) => (
            <Typography key={g} variant="body2" color="text.secondary">
              {g}
            </Typography>
          ))}
        </Box>
      )}
      {feedback && (
        <Alert severity={feedback.ok ? 'success' : 'warning'} sx={{ mt: 1 }}>
          {feedback.message}
        </Alert>
      )}
      <ConfirmDialog
        open={confirmOpen}
        title={RESET_TEXT}
        confirmLabel="Reset"
        onConfirm={() => {
          setConfirmOpen(false)
          void act(postImuReset)
        }}
        onCancel={() => setConfirmOpen(false)}
      />
    </Box>
  )
}
