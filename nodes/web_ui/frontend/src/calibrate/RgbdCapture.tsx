import { useCallback, useEffect, useRef, useState } from 'react'
import Alert from '@mui/material/Alert'
import Box from '@mui/material/Box'
import Button from '@mui/material/Button'
import Chip from '@mui/material/Chip'
import IconButton from '@mui/material/IconButton'
import Typography from '@mui/material/Typography'
import DeleteIcon from '@mui/icons-material/Delete'
import { ConfirmDialog } from '../layout/ConfirmDialog'
import { TOUCH_TARGET_PX } from '../theme'
import { CoverageGrid } from './CoverageGrid'
import {
  CaptureState,
  ComputeState,
  RgbdState,
  PoseError,
  SessionDetail,
  deletePose,
  getSession,
  poseErrors,
  startCapture,
  statusSeverity,
  stopCapture,
  thumbnailUrl,
} from './rgbdApi'

/** Caption of one pose's errors; `-` where the pose has no such view. */
const errorCaption = (e: PoseError): string => {
  const f = (v: number | null) => (v === null ? '-' : v.toFixed(3))
  return `RGB ${f(e.rgb_error_px)} / ToF ${f(e.tof_error_px)} / stereo ${f(e.stereo_error_px)} px`
}

/** Edge of one pose thumbnail in px. */
const THUMB_PX = 120

interface Props {
  capture: CaptureState
  compute: ComputeState
  /** Open session id, or null when none is open. */
  sessionId: string | null
  /** Called with the new state after a start or stop answer. */
  onState: (state: RgbdState) => void
}

/** Capture part of the RGB-D section: start/stop, live status, pose thumbnails with delete, coverage. */
export function RgbdCapture({ capture, compute, sessionId, onState }: Props) {
  const [detail, setDetail] = useState<SessionDetail | null>(null)
  const [error, setError] = useState<string | null>(null)
  const [pendingDelete, setPendingDelete] = useState<string | null>(null)

  const refresh = useCallback(async () => {
    if (sessionId === null) {
      setDetail(null)
      return
    }
    const r = await getSession(sessionId)
    if (r.ok) setDetail(r.value)
    else setError(r.message)
  }, [sessionId])

  useEffect(() => {
    void refresh()
  }, [refresh, capture.pose_count])

  // A finished compute adds a result: fetch the session again so the pose errors show.
  const wasComputing = useRef(compute.running)
  useEffect(() => {
    if (wasComputing.current && !compute.running) void refresh()
    wasComputing.current = compute.running
  }, [compute.running, refresh])

  const act = async (call: typeof stopCapture) => {
    const r = await call()
    if (r.ok) {
      setError(null)
      onState(r.value)
    } else setError(r.message)
  }

  const confirmDelete = async () => {
    const pid = pendingDelete
    setPendingDelete(null)
    if (pid === null || sessionId === null) return
    const r = await deletePose(sessionId, pid)
    if (r.ok) setError(null)
    else setError(r.message)
    await refresh()
  }

  const errors = detail ? poseErrors(detail) : new Map<string, PoseError>()
  const btn = { minHeight: TOUCH_TARGET_PX }
  return (
    <Box>
      <Box sx={{ display: 'flex', gap: 1, flexWrap: 'wrap', my: 1 }}>
        <Button variant="contained" sx={btn} disabled={capture.running || compute.running} onClick={() => void act(() => startCapture())}>
          Start capture
        </Button>
        <Button variant="outlined" sx={btn} disabled={!capture.running} onClick={() => void act(stopCapture)}>
          Stop
        </Button>
      </Box>
      {error && (
        <Alert severity="error" sx={{ mb: 1 }}>
          {error}
        </Alert>
      )}
      {capture.running && (
        <Alert severity={statusSeverity(capture.status)} sx={{ mb: 1 }}>
          <Typography>{capture.status}</Typography>
          <Typography>{`Poses: ${capture.pose_count}`}</Typography>
          {capture.stops_at && <Typography>{`Auto-stop: ${capture.stops_at}`}</Typography>}
        </Alert>
      )}
      {detail && (
        <Box>
          <Typography variant="subtitle1">{`Session ${detail.id}: ${detail.poses.length} poses`}</Typography>
          <Box sx={{ display: 'flex', flexWrap: 'wrap', gap: 1 }}>
            {detail.poses.map((p) => {
              const err = errors.get(p.id)
              const flagged = err !== undefined && err.outlier.length > 0
              return (
              <Box
                key={p.id}
                data-testid={`pose-card-${p.id}`}
                data-outlier={flagged ? 'true' : 'false'}
                sx={{ border: 1, borderColor: flagged ? 'error.main' : 'divider', p: 0.5 }}
              >
                <Box sx={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between' }}>
                  <Typography variant="caption">{p.id}</Typography>
                  <IconButton
                    aria-label={`Delete pose ${p.id}`}
                    onClick={() => setPendingDelete(p.id)}
                    sx={{ minWidth: TOUCH_TARGET_PX, minHeight: TOUCH_TARGET_PX }}
                  >
                    <DeleteIcon />
                  </IconButton>
                </Box>
                <Box sx={{ display: 'flex', gap: 0.5 }}>
                  <Box component="img" alt={`RGB ${p.id}`} src={thumbnailUrl(detail.id, p.id, 'rgb')} sx={{ width: THUMB_PX }} />
                  <Box component="img" alt={`ToF ${p.id}`} src={thumbnailUrl(detail.id, p.id, 'tof')} sx={{ width: THUMB_PX }} />
                </Box>
                <Typography variant="caption" component="div">
                  {`RGB ${p.rgb_corners} corners, ToF ${p.tof_corners} corners`}
                </Typography>
                {(p.hints ?? []).map((h) => (
                  <Typography key={h} variant="caption" component="div" color="warning.main">
                    {h}
                  </Typography>
                ))}
                {err && (
                  <Typography variant="caption" component="div">
                    {errorCaption(err)}
                  </Typography>
                )}
                {flagged && (
                  <Typography variant="caption" component="div" color="error.main">
                    high error - delete and Compute again
                  </Typography>
                )}
                {p.has_depth && <Chip size="small" label="depth" />}
              </Box>
              )
            })}
          </Box>
          <CoverageGrid rgb={detail.coverage.rgb} tof={detail.coverage.tof} />
        </Box>
      )}
      <ConfirmDialog
        open={pendingDelete !== null}
        title={`Delete pose ${pendingDelete ?? ''}?`}
        confirmLabel="Delete"
        onConfirm={() => void confirmDelete()}
        onCancel={() => setPendingDelete(null)}
      />
    </Box>
  )
}
