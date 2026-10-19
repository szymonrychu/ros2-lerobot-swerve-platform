import Box from '@mui/material/Box'
import Typography from '@mui/material/Typography'
import type { Map3dStatusPayload } from '../types'

export const NO_STATUS_TEXT = 'no 3D map status yet'
const STATE_LABELS: Record<string, string> = { waiting_for_inputs: 'waiting for inputs' }

/**
 * A finite number field of the status, or null when absent or unknown (JSON null).
 *
 * @param status - the map3d status JSON
 * @param key - field name
 * @returns the number, or null
 */
function num(status: Map3dStatusPayload, key: string): number | null {
  const v = status[key]
  return typeof v === 'number' && Number.isFinite(v) ? v : null
}

/**
 * Whether the graph correction is over the status' own thresholds (no threshold, no warning).
 *
 * @param status - the map3d status JSON
 * @returns true when translation or yaw correction exceeds its warn threshold
 */
export function correctionWarn(status: Map3dStatusPayload): boolean {
  const m = num(status, 'graph_correction_m')
  const deg = num(status, 'graph_correction_deg')
  const warnM = num(status, 'correction_warn_m')
  const warnDeg = num(status, 'correction_warn_deg')
  return (m !== null && warnM !== null && m > warnM) || (deg !== null && warnDeg !== null && deg > warnDeg)
}

/**
 * The state as shown to the user.
 *
 * @param status - the map3d status JSON
 * @returns the state label, or an empty string without one
 */
function stateLabel(status: Map3dStatusPayload): string {
  const s = status.state
  return typeof s === 'string' ? (STATE_LABELS[s] ?? s) : ''
}

/**
 * One-line summary of the 3D map status, for the layers panel.
 *
 * @param status - latest map3d status, or null/undefined before one arrived
 * @returns e.g. "mapping, 120 nodes", with ", drifted" when the correction is over its threshold
 */
export function map3dSummary(status: Map3dStatusPayload | null | undefined): string {
  if (!status) return NO_STATUS_TEXT
  const nodes = num(status, 'nodes')
  const parts = [stateLabel(status)]
  if (nodes !== null) parts.push(`${nodes} nodes`)
  if (correctionWarn(status)) parts.push('drifted')
  return parts.filter(Boolean).join(', ')
}

/**
 * The 3D map manager status: state, size, memory, age, graph correction and missing inputs.
 *
 * @param p - status: latest /map3d/status JSON, null/undefined before one arrived
 * @returns the status block
 */
export function Map3dStatus({ status }: { status: Map3dStatusPayload | null | undefined }) {
  if (!status) {
    return (
      <Typography variant="caption" color="text.secondary">
        {NO_STATUS_TEXT}
      </Typography>
    )
  }
  const nodes = num(status, 'nodes')
  const db = num(status, 'db_size_mb')
  const mem = num(status, 'memory_mb')
  const age = num(status, 'last_update_age_s')
  const corrM = num(status, 'graph_correction_m')
  const corrDeg = num(status, 'graph_correction_deg')
  const missing = Array.isArray(status.missing_inputs) ? status.missing_inputs.map(String) : []
  const facts = [
    nodes !== null && `${nodes} nodes`,
    db !== null && `db ${db.toFixed(1)} MB`,
    mem !== null && `memory ${mem.toFixed(0)} MB`,
    age !== null && `updated ${age.toFixed(1)} s ago`,
  ].filter(Boolean)
  const warn = correctionWarn(status)
  return (
    <Box data-testid="map3d-status">
      <Typography variant="caption" component="div" color={status.state === 'error' ? 'error' : 'text.primary'}>
        <span data-testid="map3d-state">{stateLabel(status)}</span>
      </Typography>
      {facts.length > 0 && (
        <Typography variant="caption" component="div" color="text.secondary">
          {facts.join(', ')}
        </Typography>
      )}
      {(corrM !== null || corrDeg !== null) && (
        <Typography
          variant="caption"
          component="div"
          data-testid="map3d-correction"
          data-warn={String(warn)}
          color={warn ? 'warning.main' : 'text.secondary'}
        >
          correction {[corrM !== null && `${corrM.toFixed(2)} m`, corrDeg !== null && `${corrDeg.toFixed(1)} deg`].filter(Boolean).join(', ')}
        </Typography>
      )}
      {status.state === 'waiting_for_inputs' && missing.length > 0 && (
        <Typography variant="caption" component="div" color="warning.main">
          missing: {missing.join(', ')}
        </Typography>
      )}
    </Box>
  )
}
