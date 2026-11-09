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
import { pairAccuracy, type ArmCameraCheck, type ArmCameraReport } from './armCameraApi'
import type { FloorParam, FloorResult } from './headFloorApi'

const M_TO_MM = 1000
const DEG_TO_RAD = Math.PI / 180
/** Distance of the floor accuracy example: a ground point this far ahead of the camera. */
const FLOOR_EXAMPLE_MM = 2000
const DEFAULT_DIGITS = 3

/**
 * One fitted parameter of a gate: value (null: not finite), 1-sigma (null: unknown), applied value and the change
 * from it (null: none applied); `held`: the gauge holds it (not fitted), shown as held in the sigma column.
 */
export interface GateParam {
  name: string
  value: number | null
  sigma: number | null
  held?: boolean
  unit: string
  applied: number | null
  change: number | null
  /** Decimals shown; DEFAULT_DIGITS when absent. */
  digits?: number
}

/** An accuracy example in plain units: its 1-sigma and 95 % values (null: unknown or not given). */
export interface GateAccuracy {
  label: string
  sigma: number | null
  p95: number | null
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
  /** Notes of the fit (gauges, model choices), shown as plain text. */
  notes: string[]
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
const ACCURACY_DIGITS = 1
/** Sigma cell of a parameter held by the gauge (not fitted). */
export const HELD_TEXT = 'held'

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
      {report.accuracy.length > 0 && <AccuracyTable accuracy={report.accuracy} />}
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
                    <TableCell>{fixed(p.value, d)}</TableCell>
                    <TableCell>{p.held ? HELD_TEXT : fixed(p.sigma, d)}</TableCell>
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
      {report.notes.length > 0 && (
        <Box sx={{ mt: 1 }} aria-label="Notes">
          <Typography variant="subtitle2" component="h4">
            Notes
          </Typography>
          {report.notes.map((n) => (
            <Typography key={n} variant="body2" color="text.secondary">
              {n}
            </Typography>
          ))}
        </Box>
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

/**
 * The accuracy examples as one row per example with its 1-sigma and 95 % columns; the 95 % column only when a value
 * of it is known.
 *
 * @param accuracy - the gate's accuracy examples
 */
function AccuracyTable({ accuracy }: { accuracy: GateAccuracy[] }) {
  const withP95 = accuracy.some((a) => a.p95 !== null)
  const cell = (v: number | null, unit: string) => (v === null ? '-' : `+-${v.toFixed(ACCURACY_DIGITS)} ${unit}`)
  return (
    <Table size="small" sx={{ mt: 1 }} aria-label="Expected accuracy">
      <TableHead>
        <TableRow>
          {['Expected accuracy', '1 sigma', ...(withP95 ? ['95 %'] : [])].map((h) => (
            <TableCell key={h}>{h}</TableCell>
          ))}
        </TableRow>
      </TableHead>
      <TableBody>
        {accuracy.map((a) => (
          <TableRow key={a.label}>
            <TableCell>{a.label}</TableCell>
            <TableCell>{cell(a.sigma, a.unit)}</TableCell>
            {withP95 && <TableCell>{cell(a.p95, a.unit)}</TableCell>}
          </TableRow>
        ))}
      </TableBody>
    </Table>
  )
}

/** A number with its unit, or '-' when it is null. */
const measured = (v: number | null, digits: number, unit: string) => (v === null ? '-' : `${v.toFixed(digits)} ${unit}`)

/**
 * Text of a refusal or warning: the node's message with the checked value against its limit.
 *
 * @param c - the refusal or warning
 * @returns "<message> (value <v> <unit>, limit <l> <unit>)"
 */
export function checkText(c: ArmCameraCheck): string {
  const digits = (v: number | null) => (v !== null && Number.isInteger(v) ? 0 : 2)
  return `${c.message} (value ${measured(c.value, digits(c.value), c.unit)}, limit ${measured(c.limit, digits(c.limit), c.unit)})`
}

/**
 * Gate of an arm-camera fit report: refusals and warnings with value against limit, the accuracy examples paired
 * (1 sigma and 95 %), params in deg and mm (a gauge-held sigma shown as held, an unknown one as a dash), the comparison of the score of the
 * calibration in effect with the new one, the tool shift examples and the fit notes.
 *
 * @param report - the report of a done result
 * @returns the gate report
 */
export function armCameraGate(report: ArmCameraReport): GateReport {
  const s = report.score
  const current = report.comparison.current_score
  return {
    plausible: report.plausible,
    refusals: report.refusals.map(checkText),
    warnings: report.warnings.map(checkText),
    score: [
      { label: 'RMS', value: measured(s.rms_px, 2, 'px') },
      { label: 'Held-out RMS', value: measured(s.heldout_rms_px, 2, 'px') },
      { label: 'Held-out tool error', value: measured(s.heldout_tool_mm, 1, 'mm') },
      { label: 'Views', value: String(s.views) },
      { label: 'Arm poses', value: String(s.poses) },
    ],
    params: report.params.map((p) => ({
      name: p.name,
      value: p.value,
      sigma: p.sigma,
      held: p.held,
      unit: p.unit,
      applied: p.applied_value,
      change: p.change,
    })),
    accuracy: pairAccuracy(report.accuracy_examples),
    comparison: [
      { label: 'RMS', current: measured(current.rms_px, 2, 'px'), next: measured(s.rms_px, 2, 'px') },
      { label: 'Held-out RMS', current: measured(current.heldout_rms_px, 2, 'px'), next: measured(s.heldout_rms_px, 2, 'px') },
      {
        label: 'Held-out tool error',
        current: measured(current.heldout_tool_mm, 1, 'mm'),
        next: measured(s.heldout_tool_mm, 1, 'mm'),
      },
      ...report.comparison.shift_examples.map((e) => ({
        label: `Shift of the ${e.label}`,
        current: '-',
        next: measured(e.value, 1, e.unit),
      })),
    ],
    notes: report.notes,
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
      { label: 'camera height', sigma: result.params.mount_z_m.sigma * M_TO_MM, p95: null, unit: 'mm' },
      {
        label: 'ground point 2 m ahead (from the pitch sigma)',
        sigma: FLOOR_EXAMPLE_MM * Math.tan(pitch),
        p95: null,
        unit: 'mm',
      },
    ],
    comparison: [],
    notes: [],
  }
}
