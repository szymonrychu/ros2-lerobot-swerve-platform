import { useCallback, useEffect, useState } from 'react'
import Alert from '@mui/material/Alert'
import Box from '@mui/material/Box'
import Typography from '@mui/material/Typography'
import { RgbdCapture } from './RgbdCapture'
import { RgbdResults } from './RgbdResults'
import { RgbdSessions } from './RgbdSessions'
import { CaptureState, ComputeState, RgbdState, getState } from './rgbdApi'

/** State poll period: 1 Hz. */
export const RGBD_POLL_MS = 1000

interface Live {
  capture: CaptureState
  compute: ComputeState
}

/** RGB-D calibration section of the Calibrate page: owns the capture/compute state and its poll. */
export function RgbdCalibration() {
  const [live, setLive] = useState<Live | null>(null)
  const [sessionId, setSessionId] = useState<string | null>(null)
  const [error, setError] = useState<string | null>(null)

  const apply = useCallback((s: RgbdState) => {
    setLive({ capture: s.capture, compute: s.compute })
    setError(null)
    if (s.capture.running && s.capture.session_id) setSessionId(s.capture.session_id)
  }, [])

  useEffect(() => {
    let alive = true
    void getState().then((r) => {
      if (!alive) return
      if (r.ok) apply(r.value)
      else setError(r.message)
    })
    return () => {
      alive = false
    }
  }, [apply])

  const reload = useCallback(async () => {
    const r = await getState()
    if (r.ok) apply(r.value)
    else setError(r.message)
  }, [apply])

  const running = live !== null && (live.capture.running || live.compute.running)

  useEffect(() => {
    if (!running) return
    let alive = true
    const poll = async () => {
      if (document.visibilityState !== 'visible') return
      const r = await getState()
      if (!alive) return
      if (r.ok) apply(r.value)
      else setError(r.message)
    }
    const id = setInterval(() => void poll(), RGBD_POLL_MS)
    const onVisible = () => void poll()
    document.addEventListener('visibilitychange', onVisible)
    return () => {
      alive = false
      clearInterval(id)
      document.removeEventListener('visibilitychange', onVisible)
    }
  }, [running, apply])

  return (
    <Box component="section" sx={{ mb: 2 }}>
      <Typography variant="h6" component="h2">
        RGB-D calibration
      </Typography>
      {error && <Alert severity="error">{error}</Alert>}
      {live && <RgbdCapture capture={live.capture} compute={live.compute} sessionId={sessionId} onState={apply} />}
      {live && (
        <RgbdSessions capture={live.capture} compute={live.compute} openId={sessionId} onOpen={setSessionId} onStarted={() => void reload()} />
      )}
      {live && <RgbdResults sessionId={sessionId} compute={live.compute} />}
    </Box>
  )
}
