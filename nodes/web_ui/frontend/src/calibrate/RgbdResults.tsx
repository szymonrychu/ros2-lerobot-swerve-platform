import { useCallback, useEffect, useRef, useState } from 'react'
import Accordion from '@mui/material/Accordion'
import AccordionDetails from '@mui/material/AccordionDetails'
import AccordionSummary from '@mui/material/AccordionSummary'
import Alert from '@mui/material/Alert'
import Box from '@mui/material/Box'
import Button from '@mui/material/Button'
import Chip from '@mui/material/Chip'
import Table from '@mui/material/Table'
import TableBody from '@mui/material/TableBody'
import TableCell from '@mui/material/TableCell'
import TableHead from '@mui/material/TableHead'
import TableRow from '@mui/material/TableRow'
import Typography from '@mui/material/Typography'
import ExpandMoreIcon from '@mui/icons-material/ExpandMore'
import { ConfirmDialog } from '../layout/ConfirmDialog'
import { TOUCH_TARGET_PX } from '../theme'
import {
  AppliedState,
  ComputeState,
  ResultSummary,
  applyResult,
  downloadUrl,
  getApplied,
  getResult,
  getSession,
} from './rgbdApi'

/** Applied-state poll period after an apply. */
export const APPLIED_POLL_MS = 2000
/** Longest wait for the unit restarts after an apply. */
export const APPLIED_POLL_MAX_MS = 60000
/** Units restarted by an apply, named in the confirmation. */
const RESTART_UNITS = ['head_camera_cpp', 'head_rgbd_cpp', 'head_state_publisher']
interface Row {
  label: string
  field: string
  limitKey: string
  limitPrefix: string
  unit: string
}

const ROWS: Row[] = [
  { label: 'RGB RMS', field: 'rms_px', limitKey: 'intrinsics_max_rms_px', limitPrefix: '<=', unit: 'px' },
  { label: 'Focal difference', field: 'focal_diff_pct', limitKey: 'intrinsics_max_focal_diff_pct', limitPrefix: '<=', unit: '%' },
  {
    label: 'Principal point difference',
    field: 'principal_diff_px',
    limitKey: 'intrinsics_max_principal_diff_px',
    limitPrefix: '<=',
    unit: 'px',
  },
  { label: 'Plane bias', field: 'plane_bias_median_mm', limitKey: 'plane_max_bias_mm', limitPrefix: '|bias| <=', unit: 'mm' },
  { label: 'Plane spread', field: 'plane_bias_spread_mm', limitKey: 'plane_max_spread_mm', limitPrefix: '<=', unit: 'mm' },
  { label: 'RGB views', field: 'views', limitKey: 'min_views', limitPrefix: '>=', unit: '' },
]

const isRec = (v: unknown): v is Record<string, unknown> => typeof v === 'object' && v !== null

/** Limit as recorded in the result's own `limits` object; undefined when the result carries none. */
function limitOf(result: ResultSummary, key: string): number | undefined {
  const own = isRec(result.limits) ? result.limits[key] : undefined
  return typeof own === 'number' ? own : undefined
}

/** Metrics value as shown: numbers trimmed to 2 decimals, missing as a dash. */
function show(v: unknown): string {
  if (typeof v !== 'number') return '-'
  return Number.isInteger(v) ? String(v) : String(Math.round(v * 100) / 100)
}

const failedLimits = (r: ResultSummary): string[] =>
  Array.isArray(r.failed_limits) ? r.failed_limits.filter((x): x is string => typeof x === 'string') : []

interface Props {
  /** Open session id, or null. */
  sessionId: string | null
  compute: ComputeState
}

/** Results of the open session, newest first, with report, Apply and Download. */
export function RgbdResults({ sessionId, compute }: Props) {
  const [results, setResults] = useState<ResultSummary[]>([])
  const [reports, setReports] = useState<Record<string, string>>({})
  const [error, setError] = useState<string | null>(null)
  const [pendingApply, setPendingApply] = useState<string | null>(null)
  const [applied, setApplied] = useState<AppliedState | null>(null)
  const [applying, setApplying] = useState(false)
  const poll = useRef<ReturnType<typeof setInterval> | null>(null)

  const refresh = useCallback(async () => {
    if (sessionId === null) {
      setResults([])
      return
    }
    const r = await getSession(sessionId)
    if (r.ok) {
      setResults([...r.value.results].sort((a, b) => (String(a.created ?? a.id) < String(b.created ?? b.id) ? 1 : -1)))
      setError(null)
    } else setError(r.message)
  }, [sessionId])

  useEffect(() => {
    void refresh()
  }, [refresh])

  const wasComputing = useRef(false)
  useEffect(() => {
    if (wasComputing.current && !compute.running) void refresh()
    wasComputing.current = compute.running
  }, [compute.running, refresh])

  const stopPoll = useCallback(() => {
    if (poll.current !== null) clearInterval(poll.current)
    poll.current = null
    setApplying(false)
  }, [])
  useEffect(() => stopPoll, [stopPoll])

  const loadReport = async (rid: string) => {
    if (sessionId === null || reports[rid] !== undefined) return
    const r = await getResult(sessionId, rid)
    if (r.ok) setReports((prev) => ({ ...prev, [rid]: r.value.report ?? '(no report)' }))
    else setError(r.message)
  }

  const confirmApply = async () => {
    const rid = pendingApply
    setPendingApply(null)
    if (rid === null || sessionId === null) return
    const r = await applyResult(sessionId, rid)
    if (!r.ok) {
      setError(r.message)
      return
    }
    setError(null)
    setApplied(r.value)
    stopPoll()
    setApplying(true)
    const started = Date.now()
    poll.current = setInterval(() => {
      void getApplied().then((a) => {
        if (a.ok) {
          setApplied(a.value)
          if (a.value.units.every((u) => u.restarted)) stopPoll()
        } else setError(a.message)
      })
      if (Date.now() - started >= APPLIED_POLL_MAX_MS) stopPoll()
    }, APPLIED_POLL_MS)
  }

  if (sessionId === null) return null
  const btn = { minHeight: TOUCH_TARGET_PX }
  return (
    <Box sx={{ my: 1 }}>
      <Typography variant="subtitle1">{`Results of session ${sessionId}`}</Typography>
      {error && (
        <Alert severity="error" sx={{ mb: 1 }}>
          {error}
        </Alert>
      )}
      {results.length === 0 && <Typography variant="body2">No results yet.</Typography>}
      {results.map((res) => (
        <Box key={res.id} sx={{ border: 1, borderColor: 'divider', p: 1, mb: 1 }}>
          <Box sx={{ display: 'flex', alignItems: 'center', gap: 1, flexWrap: 'wrap' }}>
            <Typography>{res.id}</Typography>
            <Chip
              data-testid="result-chip"
              size="small"
              color={res.passed ? 'success' : 'error'}
              label={res.passed ? 'passed' : 'failed'}
            />
            {res.created !== undefined && <Typography variant="caption">{String(res.created)}</Typography>}
            {res.passed && (
              <Button variant="contained" sx={btn} disabled={applying} onClick={() => setPendingApply(res.id)}>
                Apply
              </Button>
            )}
            <Button component="a" variant="outlined" sx={btn} href={downloadUrl(sessionId, res.id)} download>
              Download
            </Button>
          </Box>
          {typeof res.error === 'string' && <Alert severity="error">{res.error}</Alert>}
          <Table size="small">
            <TableHead>
              <TableRow>
                <TableCell>Item</TableCell>
                <TableCell>Value</TableCell>
                <TableCell>Limit</TableCell>
              </TableRow>
            </TableHead>
            <TableBody>
              {ROWS.map((row) => {
                const lim = limitOf(res, row.limitKey)
                return (
                  <TableRow key={row.label}>
                    <TableCell>{row.label}</TableCell>
                    <TableCell>{`${show(res[row.field])}`}</TableCell>
                    <TableCell>{lim === undefined ? '-' : `${row.limitPrefix} ${lim}${row.unit ? ` ${row.unit}` : ''}`}</TableCell>
                  </TableRow>
                )
              })}
            </TableBody>
          </Table>
          {failedLimits(res).length > 0 && (
            <Alert severity="warning" sx={{ mt: 1 }}>
              {failedLimits(res).map((f) => (
                <div key={f}>{f}</div>
              ))}
            </Alert>
          )}
          <Accordion disableGutters onChange={(_, open) => open && void loadReport(res.id)}>
            <AccordionSummary expandIcon={<ExpandMoreIcon />}>Report</AccordionSummary>
            <AccordionDetails>
              <Box component="pre" sx={{ whiteSpace: 'pre-wrap', m: 0 }}>
                {reports[res.id] ?? 'Loading...'}
              </Box>
            </AccordionDetails>
          </Accordion>
        </Box>
      ))}
      {applied && (
        <Alert severity={applied.units.every((u) => u.restarted) ? 'success' : 'info'} sx={{ mt: 1 }}>
          <div>{`Applied result ${applied.result} of session ${applied.session}`}</div>
          {applied.units.map((u) => (
            <div key={u.name}>{`${u.name}: ${u.restarted ? 'restarted' : 'restarting'}`}</div>
          ))}
        </Alert>
      )}
      <ConfirmDialog
        open={pendingApply !== null}
        title={`Apply result ${pendingApply ?? ''}?`}
        body={`Applying restarts ${RESTART_UNITS.join(', ')} on the robot.`}
        confirmLabel="Apply"
        onConfirm={() => void confirmApply()}
        onCancel={() => setPendingApply(null)}
      />
    </Box>
  )
}
