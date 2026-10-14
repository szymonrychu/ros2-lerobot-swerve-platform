import { useCallback, useEffect, useRef, useState } from 'react'
import Alert from '@mui/material/Alert'
import Box from '@mui/material/Box'
import Button from '@mui/material/Button'
import Typography from '@mui/material/Typography'
import { ConfirmDialog } from '../layout/ConfirmDialog'
import { TOUCH_TARGET_PX } from '../theme'
import {
  ApiResult,
  CaptureList,
  PreviewInfo,
  SIDES,
  SolveStatus,
  capturableColor,
  captureThumbUrl,
  deleteCaptures,
  fetchCaptures,
  fetchPreview,
  fetchSolve,
  isFinalSolveState,
  postCapture,
  postSolve,
  previewImageUrl,
} from './stereoApi'

/** Preview refresh period: at most 2 Hz. */
export const PREVIEW_INTERVAL_MS = 500
export const SOLVE_POLL_MS = 2000
const THUMB_WIDTH = '5rem'
const REFUSED_TEXT = 'rms above the gate - previous calibration kept'
const DELETE_TEXT = 'Delete all captures and the active calibration? Depth stops until you solve again.'

type PreviewState = { kind: 'ok'; info: PreviewInfo } | { kind: 'error'; message: string } | { kind: 'loading' }

function fmt(n: number | null | undefined): string {
  return n === null || n === undefined ? 'n/a' : String(n)
}

function SolveResult({ solve }: { solve: SolveStatus }) {
  return (
    <Box sx={{ mt: 1 }}>
      <Typography>State: {solve.state}</Typography>
      {solve.state !== 'idle' && solve.state !== 'running' && (
        <>
          <Typography>
            rms left {fmt(solve.rms_left)} / right {fmt(solve.rms_right)} / stereo {fmt(solve.rms)}
          </Typography>
          <Typography>
            Pairs: {fmt(solve.pairs_used)} used, {fmt(solve.pairs_skipped)} skipped
          </Typography>
        </>
      )}
      {solve.state === 'refused' ? (
        <Typography color="warning.main">{REFUSED_TEXT}</Typography>
      ) : (
        solve.message && <Typography>{solve.message}</Typography>
      )}
    </Box>
  )
}

/** Stereo calibration: live preview with detection quality, pair capture, captures list, delete and solve. */
export function StereoCalibration() {
  const [preview, setPreview] = useState<PreviewState>({ kind: 'loading' })
  const [captures, setCaptures] = useState<CaptureList | null>(null)
  const [listError, setListError] = useState<string | null>(null)
  const [captureMsg, setCaptureMsg] = useState<{ ok: boolean; text: string } | null>(null)
  const [busy, setBusy] = useState(false)
  const [solve, setSolve] = useState<SolveStatus | null>(null)
  const [solveError, setSolveError] = useState<string | null>(null)
  const [confirmOpen, setConfirmOpen] = useState(false)
  const live = useRef(true)

  const refreshCaptures = useCallback(async () => {
    const r = await fetchCaptures()
    if (!live.current) return
    if (r.ok) {
      setCaptures(r.value)
      setListError(null)
    } else setListError(r.message)
  }, [])

  // Preview: refreshed together (one JSON, then both images by its seq), only while the page is visible.
  useEffect(() => {
    live.current = true
    let timer: ReturnType<typeof setTimeout> | undefined
    const tick = async () => {
      timer = undefined
      if (document.visibilityState !== 'visible') return
      const r = await fetchPreview()
      if (!live.current) return
      setPreview(r.ok ? { kind: 'ok', info: r.value } : { kind: 'error', message: r.message })
      if (document.visibilityState === 'visible') timer = setTimeout(() => void tick(), PREVIEW_INTERVAL_MS)
    }
    const onVisibility = () => {
      if (document.visibilityState === 'visible' && timer === undefined) void tick()
    }
    document.addEventListener('visibilitychange', onVisibility)
    void tick()
    return () => {
      live.current = false
      if (timer !== undefined) clearTimeout(timer)
      document.removeEventListener('visibilitychange', onVisibility)
    }
  }, [])

  const solveRunning = solve?.state === 'running'

  // Solve polling every 2 s until the state is final; also resumes a job already running on the Jetson.
  useEffect(() => {
    void refreshCaptures()
    void fetchSolve().then((r) => {
      if (live.current && r.ok) setSolve(r.value)
    })
  }, [refreshCaptures])

  useEffect(() => {
    if (!solveRunning) return
    const id = setInterval(() => {
      void fetchSolve().then((r) => {
        if (!live.current) return
        if (!r.ok) {
          setSolveError(r.message)
          return
        }
        setSolveError(null)
        setSolve(r.value)
        if (isFinalSolveState(r.value.state)) void refreshCaptures()
      })
    }, SOLVE_POLL_MS)
    return () => clearInterval(id)
  }, [solveRunning, refreshCaptures])

  const run = async <T,>(action: () => Promise<ApiResult<T>>): Promise<ApiResult<T>> => {
    setBusy(true)
    try {
      return await action()
    } finally {
      if (live.current) setBusy(false)
    }
  }

  const capture = async () => {
    const r = await run(postCapture)
    if (!live.current) return
    setCaptureMsg(r.ok ? { ok: true, text: `Captured - ${r.value.captures} pairs` } : { ok: false, text: r.message })
    if (r.ok) void refreshCaptures()
  }

  const doDelete = async () => {
    setConfirmOpen(false)
    const r = await run(deleteCaptures)
    if (!live.current) return
    setCaptureMsg(r.ok ? { ok: true, text: `Deleted ${r.value.removed_captures} captures` } : { ok: false, text: r.message })
    setSolve(null)
    void refreshCaptures()
  }

  const startSolve = async () => {
    setSolveError(null)
    const r = await run(postSolve)
    if (!live.current) return
    if (r.ok) setSolve(r.value)
    else setSolveError(r.message)
  }

  const count = captures?.count ?? 0
  const minPairs = captures?.min_pairs_to_solve ?? (preview.kind === 'ok' ? preview.info.min_pairs_to_solve : 0)
  const blocked = busy || solveRunning
  const btn = { minHeight: TOUCH_TARGET_PX, minWidth: TOUCH_TARGET_PX }

  return (
    <Box component="section" sx={{ mb: 3 }}>
      <Typography variant="h6" component="h2">
        Stereo calibration
      </Typography>

      {preview.kind === 'error' && <Alert severity="warning">{preview.message}</Alert>}
      {preview.kind === 'ok' && (
        <Box sx={{ display: 'flex', flexDirection: { xs: 'column', sm: 'row' }, alignItems: 'center', gap: 1, my: 1 }}>
          {SIDES.map((side, i) => (
            <Box key={side} sx={{ display: 'contents' }}>
              <Box sx={{ flex: 1, textAlign: 'center', minWidth: 0 }}>
                <Box
                  component="img"
                  src={previewImageUrl(side, preview.info.seq)}
                  alt={`${side} camera`}
                  sx={{ width: '100%', maxWidth: '24rem' }}
                />
                <Typography>{`corners ${preview.info[side].corners}`}</Typography>
              </Box>
              {i === 0 && (
                <Typography sx={{ color: capturableColor(preview.info.capturable), fontWeight: 'bold' }}>
                  {`shared ${preview.info.shared_corners} / needed ${preview.info.min_shared_corners}`}
                </Typography>
              )}
            </Box>
          ))}
        </Box>
      )}

      <Box sx={{ display: 'flex', gap: 1, flexWrap: 'wrap', my: 1 }}>
        <Button variant="contained" sx={btn} disabled={blocked} onClick={() => void capture()}>
          Capture pair
        </Button>
        <Button variant="contained" sx={btn} disabled={blocked || count < minPairs || captures === null} onClick={() => void startSolve()}>
          Solve
        </Button>
        <Button variant="outlined" color="error" sx={btn} disabled={blocked} onClick={() => setConfirmOpen(true)}>
          Delete captures
        </Button>
      </Box>

      {captureMsg && <Alert severity={captureMsg.ok ? 'success' : 'error'}>{captureMsg.text}</Alert>}
      {solveError && <Alert severity="error">{solveError}</Alert>}
      {solve && <SolveResult solve={solve} />}

      <Typography sx={{ mt: 1 }}>{`${count} / ${minPairs} pairs`}</Typography>
      {listError && <Alert severity="error">{listError}</Alert>}
      <Box sx={{ display: 'flex', flexDirection: 'column', gap: 0.5 }}>
        {[...(captures?.captures ?? [])].reverse().map((c) => (
          <Box key={c.id} sx={{ display: 'flex', gap: 0.5, alignItems: 'center' }}>
            {SIDES.map((side) => (
              <Box
                key={side}
                component="img"
                src={captureThumbUrl(c.id, side)}
                alt={`capture ${c.id} ${side}`}
                sx={{ width: THUMB_WIDTH }}
              />
            ))}
            <Typography variant="body2">{c.id}</Typography>
          </Box>
        ))}
      </Box>

      <ConfirmDialog
        open={confirmOpen}
        title={DELETE_TEXT}
        confirmLabel="Delete"
        onConfirm={() => void doDelete()}
        onCancel={() => setConfirmOpen(false)}
      />
    </Box>
  )
}
