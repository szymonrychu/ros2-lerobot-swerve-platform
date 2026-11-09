import { useState } from 'react'
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
import { ConfirmDialog } from '../layout/ConfirmDialog'
import { TOUCH_TARGET_PX } from '../theme'
import type { ArmCameraReport } from './armCameraApi'
import type { FloorParam, FloorResult } from './headFloorApi'

const M_TO_MM = 1000
const DEG_TO_RAD = Math.PI / 180
/** Distance of the floor accuracy example: a ground point this far ahead of the camera. */
const FLOOR_EXAMPLE_MM = 2000
const DEFAULT_DIGITS = 3

/** One fitted parameter of a gate: value, 1-sigma, applied value and the change from it (null: none applied). */
export interface GateParam {
  name: string
  value: number
  sigma: number | null
  unit: string
  applied: number | null
  change: number | null
  /** Decimals shown; DEFAULT_DIGITS when absent. */
  digits?: number
}

/** An accuracy example in plain units, shown as "<label>: +-<value> <unit>". */
export interface GateAccuracy {
  label: string
  value: number
  unit: string
}

/**
 * Everything a quality gate shows. `plausible` false disables Accept; `refusals` say why. The flows map their own
 * results into this shape (armCameraGate, floorGate, the servo and IMU checks).
 */
export interface GateReport {
  plausible: boolean
  refusals: string[]
  warnings: string[]
  score: { label: string; value: string }[]
  params: GateParam[]
  accuracy: GateAccuracy[]
  comparison: { label: string; current: string; next: string }[]
}

export type GateDecision = 'accepted' | 'declined' | null

interface Props {
  report: GateReport
  /** Body of the Accept confirm: what accepting writes or applies. */
  acceptBody: string
  onAccept: () => void
  onDecline: () => void
  /** The decision taken; both buttons are off once there is one. */
  decision: GateDecision
  busy?: boolean
}

const signed = (v: number, digits: number) => `${v >= 0 ? '+' : ''}${v.toFixed(digits)}`
const fixed = (v: number | null, digits: number) => (v === null ? '-' : v.toFixed(digits))

/**
 * The quality gate that ends every calibration flow: plausibility with the refusal reasons, warnings, scores, the
 * fitted parameters with sigma and change from the applied values, accuracy examples in plain units and the
 * comparison with the current calibration; Accept (confirmed) and Decline.
 *
 * @param props - see Props
 */
export function QualityGate({ report, acceptBody, onAccept, onDecline, decision, busy = false }: Props) {
  const [confirm, setConfirm] = useState(false)
  const target = { minHeight: TOUCH_TARGET_PX }
  const decided = decision !== null
  return (
    <Box sx={{ mt: 1 }}>
      <Box sx={{ display: 'flex', gap: 1, alignItems: 'center', flexWrap: 'wrap' }}>
        <Chip size="small" label={report.plausible ? 'Plausible' : 'Not plausible'} color={report.plausible ? 'success' : 'error'} />
        {decision && <Chip size="small" variant="outlined" label={decision === 'accepted' ? 'Accepted' : 'Declined'} />}
      </Box>
      {report.refusals.map((r) => (
        <Alert key={r} severity="error" sx={{ mt: 1 }}>
          {r}
        </Alert>
      ))}
      {report.warnings.map((w) => (
        <Alert key={w} severity="warning" sx={{ mt: 1 }}>
          {w}
        </Alert>
      ))}
      {report.score.length > 0 && (
        <Table size="small" sx={{ mt: 1 }} aria-label="Score">
          <TableBody>
            {report.score.map((s) => (
              <TableRow key={s.label}>
                <TableCell>{s.label}</TableCell>
                <TableCell>{s.value}</TableCell>
              </TableRow>
            ))}
          </TableBody>
        </Table>
      )}
      {report.accuracy.length > 0 && (
        <Box sx={{ mt: 1 }}>
          <Typography variant="subtitle2" component="h4">
            Expected accuracy
          </Typography>
          {report.accuracy.map((a) => (
            <Typography key={a.label} variant="body2">{`${a.label}: +-${a.value.toFixed(1)} ${a.unit}`}</Typography>
          ))}
        </Box>
      )}
      {report.params.length > 0 && (
        <Box sx={{ overflowX: 'auto' }}>
          <Table size="small" sx={{ mt: 1 }} aria-label="Parameters">
            <TableHead>
              <TableRow>
                {['Parameter', 'Unit', 'New', 'Sigma', 'Applied', 'Change'].map((h) => (
                  <TableCell key={h}>{h}</TableCell>
                ))}
              </TableRow>
            </TableHead>
            <TableBody>
              {report.params.map((p) => {
                const d = p.digits ?? DEFAULT_DIGITS
                return (
                  <TableRow key={p.name}>
                    <TableCell>{p.name}</TableCell>
                    <TableCell>{p.unit}</TableCell>
                    <TableCell>{p.value.toFixed(d)}</TableCell>
                    <TableCell>{fixed(p.sigma, d)}</TableCell>
                    <TableCell>{fixed(p.applied, d)}</TableCell>
                    <TableCell>{p.change === null ? '-' : signed(p.change, d)}</TableCell>
                  </TableRow>
                )
              })}
            </TableBody>
          </Table>
        </Box>
      )}
      {report.comparison.length > 0 && (
        <Table size="small" sx={{ mt: 1 }} aria-label="Comparison with the current calibration">
          <TableHead>
            <TableRow>
              {['Compared', 'Current', 'New'].map((h) => (
                <TableCell key={h}>{h}</TableCell>
              ))}
            </TableRow>
          </TableHead>
          <TableBody>
            {report.comparison.map((c) => (
              <TableRow key={c.label}>
                <TableCell>{c.label}</TableCell>
                <TableCell>{c.current}</TableCell>
                <TableCell>{c.next}</TableCell>
              </TableRow>
            ))}
          </TableBody>
        </Table>
      )}
      <Box sx={{ display: 'flex', gap: 1, mt: 1, flexWrap: 'wrap' }}>
        <Button variant="contained" sx={target} disabled={busy || decided || !report.plausible} onClick={() => setConfirm(true)}>
          Accept
        </Button>
        <Button variant="outlined" sx={target} disabled={busy || decided} onClick={onDecline}>
          Decline
        </Button>
      </Box>
      {!report.plausible && !decided && (
        <Typography variant="body2" color="text.secondary">
          Accept is off: the result is not plausible (reasons above). Decline it and run the flow again.
        </Typography>
      )}
      <ConfirmDialog
        open={confirm}
        title="Accept this calibration?"
        body={acceptBody}
        confirmLabel="Accept"
        onCancel={() => setConfirm(false)}
        onConfirm={() => {
          setConfirm(false)
          onAccept()
        }}
      />
    </Box>
  )
}

/** Text of one comparison value. */
function comparisonText(v: unknown): string {
  if (typeof v === 'number') return String(v)
  if (typeof v === 'string') return v
  return v === null || v === undefined ? '-' : JSON.stringify(v)
}

/**
 * Gate of an arm-camera fit report.
 *
 * @param report - the report of a done result
 * @returns the gate report
 */
export function armCameraGate(report: ArmCameraReport): GateReport {
  const s = report.score
  return {
    plausible: report.plausible,
    refusals: report.refusals,
    warnings: report.warnings,
    score: [
      { label: 'RMS', value: `${s.rms_px.toFixed(2)} px` },
      { label: 'Held-out RMS', value: `${s.heldout_rms_px.toFixed(2)} px` },
      { label: 'Held-out tool error', value: `${s.heldout_tool_mm.toFixed(1)} mm` },
      { label: 'Views', value: String(s.views) },
      { label: 'Arm poses', value: String(s.poses) },
    ],
    params: report.params.map((p) => ({
      name: p.name,
      value: p.value,
      sigma: p.sigma,
      unit: p.unit,
      applied: p.applied_value,
      change: p.change,
    })),
    accuracy: report.accuracy_examples,
    comparison: Object.entries(report.comparison).map(([label, v]) => {
      const pair = typeof v === 'object' && v !== null && 'current' in v && 'new' in v ? (v as { current: unknown; new: unknown }) : null
      return pair
        ? { label, current: comparisonText(pair.current), next: comparisonText(pair.new) }
        : { label, current: '-', next: comparisonText(v) }
    }),
  }
}

interface FloorRow {
  name: string
  key: keyof FloorResult['params']
  unit: string
  scale: number
  digits: number
}

const FLOOR_ROWS: FloorRow[] = [
  { name: 'Mount z', key: 'mount_z_m', unit: 'mm', scale: M_TO_MM, digits: 1 },
  { name: 'Mount roll', key: 'mount_roll_deg', unit: 'deg', scale: 1, digits: 3 },
  { name: 'Mount pitch', key: 'mount_pitch_deg', unit: 'deg', scale: 1, digits: 3 },
  { name: 'Tilt zero', key: 'tilt_zero_deg', unit: 'deg', scale: 1, digits: 3 },
  { name: 'Tilt roll', key: 'tilt_roll_deg', unit: 'deg', scale: 1, digits: 3 },
]

/**
 * Gate of a head floor fit: its limits are the plausibility, the URDF values are the applied ones.
 *
 * @param result - floor fit result
 * @returns the gate report
 */
export function floorGate(result: FloorResult): GateReport {
  const rows = FLOOR_ROWS.flatMap((row) => {
    const p: FloorParam | undefined = result.params[row.key]
    return p
      ? [
          {
            name: row.name,
            unit: row.unit,
            value: p.value * row.scale,
            sigma: p.sigma * row.scale,
            applied: p.urdf * row.scale,
            change: (p.value - p.urdf) * row.scale,
            digits: row.digits,
          },
        ]
      : []
  })
  const pitch = Math.hypot(result.params.mount_pitch_deg.sigma, result.params.tilt_zero_deg.sigma) * DEG_TO_RAD
  return {
    plausible: result.passed,
    refusals: result.failed_limits,
    warnings: [],
    score: [
      { label: 'Height RMS (pose mean)', value: `${result.height_rms_mm} mm` },
      { label: 'Normal RMS (pose mean)', value: `${result.normal_rms_deg} deg` },
      { label: 'Poses', value: String(result.poses.length) },
      { label: 'Skipped poses', value: String(result.skipped.length) },
    ],
    params: rows,
    accuracy: [
      { label: 'camera height', value: result.params.mount_z_m.sigma * M_TO_MM, unit: 'mm' },
      { label: 'ground point 2 m ahead (from the pitch sigma)', value: FLOOR_EXAMPLE_MM * Math.tan(pitch), unit: 'mm' },
    ],
    comparison: [],
  }
}
