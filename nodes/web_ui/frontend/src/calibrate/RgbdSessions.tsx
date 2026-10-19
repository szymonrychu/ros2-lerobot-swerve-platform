import { useCallback, useEffect, useRef, useState } from 'react'
import Alert from '@mui/material/Alert'
import Box from '@mui/material/Box'
import Button from '@mui/material/Button'
import Table from '@mui/material/Table'
import TableBody from '@mui/material/TableBody'
import TableCell from '@mui/material/TableCell'
import TableHead from '@mui/material/TableHead'
import TableRow from '@mui/material/TableRow'
import Typography from '@mui/material/Typography'
import { TOUCH_TARGET_PX } from '../theme'
import { CaptureState, ComputeState, SessionSummary, computeSession, listSessions, startCapture } from './rgbdApi'

interface Props {
  capture: CaptureState
  compute: ComputeState
  /** Id of the open session, or null. */
  openId: string | null
  onOpen: (id: string) => void
  /** Called after a capture resume or a compute was accepted, so the root polls. */
  onStarted: () => void
}

/** Newest first; created is an ISO string or an epoch number. */
const byNewest = (a: SessionSummary, b: SessionSummary) => (a.created < b.created ? 1 : a.created > b.created ? -1 : 0)

/** Session list of the RGB-D section: open, resume capture, compute. */
export function RgbdSessions({ capture, compute, openId, onOpen, onStarted }: Props) {
  const [sessions, setSessions] = useState<SessionSummary[]>([])
  const [error, setError] = useState<string | null>(null)

  const refresh = useCallback(async () => {
    const r = await listSessions()
    if (r.ok) setSessions([...r.value].sort(byNewest))
    else setError(r.message)
  }, [])

  useEffect(() => {
    void refresh()
  }, [refresh, capture.running])

  const wasComputing = useRef(false)
  useEffect(() => {
    if (wasComputing.current && !compute.running) void refresh()
    wasComputing.current = compute.running
  }, [compute.running, refresh])

  const busy = capture.running || compute.running

  const resume = async (id: string) => {
    const r = await startCapture(id)
    if (r.ok) {
      setError(null)
      onStarted()
    } else setError(r.message)
  }

  const run = async (id: string) => {
    const r = await computeSession(id)
    if (r.ok) {
      setError(null)
      onStarted()
    } else setError(r.message)
  }

  const btn = { minHeight: TOUCH_TARGET_PX }
  return (
    <Box sx={{ my: 1 }}>
      <Typography variant="subtitle1">Sessions</Typography>
      {error && (
        <Alert severity="error" sx={{ mb: 1 }}>
          {error}
        </Alert>
      )}
      {compute.error && (
        <Alert severity="error" sx={{ mb: 1 }}>{`Compute failed: ${compute.error}`}</Alert>
      )}
      <Table size="small">
        <TableHead>
          <TableRow>
            <TableCell>Session</TableCell>
            <TableCell>Created</TableCell>
            <TableCell>Poses</TableCell>
            <TableCell>Results</TableCell>
            <TableCell />
          </TableRow>
        </TableHead>
        <TableBody>
          {sessions.map((s) => (
            <TableRow key={s.id} selected={s.id === openId}>
              <TableCell>{s.id}</TableCell>
              <TableCell>{String(s.created)}</TableCell>
              <TableCell>{s.pose_count}</TableCell>
              <TableCell>{s.result_count}</TableCell>
              <TableCell>
                <Box sx={{ display: 'flex', gap: 1, flexWrap: 'wrap' }}>
                  <Button sx={btn} aria-label={`Open ${s.id}`} onClick={() => onOpen(s.id)}>
                    Open
                  </Button>
                  <Button sx={btn} aria-label={`Resume capture ${s.id}`} disabled={busy} onClick={() => void resume(s.id)}>
                    Resume capture
                  </Button>
                  <Button sx={btn} aria-label={`Compute ${s.id}`} disabled={busy} onClick={() => void run(s.id)}>
                    Compute
                  </Button>
                </Box>
              </TableCell>
            </TableRow>
          ))}
        </TableBody>
      </Table>
    </Box>
  )
}
