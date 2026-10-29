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
import { AppliedRecord, CheckLimits, ComputeState, ResultSummary, applyResult, downloadUrl, getResult, getSession } from './rgbdApi'

/** Explanation in the Accept check confirmation: accepting records the check and changes nothing on the robot. */
const ACCEPT_BODY =
  'Records this passed check as the accepted D435 intrinsics check. The D435 factory intrinsics stay in use; nothing on the robot changes.'

type Metric = 'rms_px' | 'focal_diff_pct' | 'principal_diff_px' | 'plane_bias_median_mm' | 'plane_bias_spread_mm' | 'views'
type Verdict = 'pass' | 'fail' | '-'

interface Row {
  label: string
  field: Metric
  limitKey: keyof CheckLimits
  limitPrefix: string
  unit: string
}

const ROWS: Row[] = [
  { label: 'Colour RMS', field: 'rms_px', limitKey: 'intrinsics_max_rms_px', limitPrefix: '<=', unit: 'px' },
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
  { label: 'Colour views', field: 'views', limitKey: 'min_views', limitPrefix: '>=', unit: '' },
]

/** Metrics value as shown: numbers trimmed to 2 decimals, unmeasured as a dash. */
function show(v: number | null): string {
  if (v === null) return '-'
  return Number.isInteger(v) ? String(v) : String(Math.round(v * 100) / 100)
}

/**
 * Pass/fail of one limit of a check.
 *
 * @param result the check result.
 * @param row the row of the limit.
 * @returns 'fail' when a failed_limits entry names the row's `limits.<key>`, '-' when the value was not measured, else 'pass'.
 */
function verdict(result: ResultSummary, row: Row): Verdict {
  if (result.failed_limits.some((f) => f.startsWith(`limits.${row.limitKey}:`))) return 'fail'
  return result[row.field] === null ? '-' : 'pass'
}

interface Props {
  /** Open session id, or null. */
  sessionId: string | null
  compute: ComputeState
}

/** Check results of the open session, newest first, with the per-limit rows, report, Accept check and Download. */
export function RgbdResults({ sessionId, compute }: Props) {
  const [results, setResults] = useState<ResultSummary[]>([])
  const [reports, setReports] = useState<Record<string, string>>({})
  const [error, setError] = useState<string | null>(null)
  const [pendingAccept, setPendingAccept] = useState<string | null>(null)
  const [accepted, setAccepted] = useState<AppliedRecord | null>(null)
  const [accepting, setAccepting] = useState(false)

  const refresh = useCallback(async () => {
    if (sessionId === null) {
      setResults([])
      return
    }
    const r = await getSession(sessionId)
    if (r.ok) {
      setResults([...r.value.results].sort((a, b) => (a.created < b.created ? 1 : -1)))
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

  const loadReport = async (rid: string) => {
    if (sessionId === null || reports[rid] !== undefined) return
    const r = await getResult(sessionId, rid)
    if (r.ok) setReports((prev) => ({ ...prev, [rid]: r.value.report ?? '(no report)' }))
    else setError(r.message)
  }

  // Accepting records the check in applied.json and restarts nothing, so the answer is final: no wait follows it.
  const confirmAccept = async () => {
    const rid = pendingAccept
    setPendingAccept(null)
    if (rid === null || sessionId === null) return
    setAccepting(true)
    const r = await applyResult(sessionId, rid)
    setAccepting(false)
    if (r.ok) {
      setError(null)
      setAccepted(r.value)
    } else setError(r.message)
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
        <Box key={res.id} data-testid={`result-card-${res.id}`} sx={{ border: 1, borderColor: 'divider', p: 1, mb: 1 }}>
          <Box sx={{ display: 'flex', alignItems: 'center', gap: 1, flexWrap: 'wrap' }}>
            <Typography>{res.id}</Typography>
            <Chip
              data-testid="result-chip"
              size="small"
              color={res.passed ? 'success' : 'error'}
              label={res.passed ? 'passed' : 'failed'}
            />
            <Typography variant="caption">{res.created}</Typography>
            <Button variant="contained" sx={btn} disabled={!res.passed || accepting} onClick={() => setPendingAccept(res.id)}>
              Accept check
            </Button>
            <Button component="a" variant="outlined" sx={btn} href={downloadUrl(sessionId, res.id)} download>
              Download
            </Button>
          </Box>
          {res.error !== undefined && <Alert severity="error">{res.error}</Alert>}
          <Table size="small">
            <TableHead>
              <TableRow>
                <TableCell>Item</TableCell>
                <TableCell>Value</TableCell>
                <TableCell>Limit</TableCell>
                <TableCell>Check</TableCell>
              </TableRow>
            </TableHead>
            <TableBody>
              {ROWS.map((row) => (
                <TableRow key={row.label}>
                  <TableCell>{row.label}</TableCell>
                  <TableCell>{show(res[row.field])}</TableCell>
                  <TableCell>{`${row.limitPrefix} ${res.limits[row.limitKey]}${row.unit ? ` ${row.unit}` : ''}`}</TableCell>
                  <TableCell>{verdict(res, row)}</TableCell>
                </TableRow>
              ))}
            </TableBody>
          </Table>
          {res.failed_limits.length > 0 && (
            <Alert severity="warning" sx={{ mt: 1 }}>
              {res.failed_limits.map((f) => (
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
      {accepted && (
        <Alert severity="success" sx={{ mt: 1 }}>
          {`Accepted check ${accepted.result} of session ${accepted.session}`}
        </Alert>
      )}
      <ConfirmDialog
        open={pendingAccept !== null}
        title={`Accept check ${pendingAccept ?? ''}?`}
        body={ACCEPT_BODY}
        confirmLabel="Accept check"
        onConfirm={() => void confirmAccept()}
        onCancel={() => setPendingAccept(null)}
      />
    </Box>
  )
}
