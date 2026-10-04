/** Pure battery chip / banner logic: level and colour from the backend battery payload and thresholds. */

export interface BatteryConfig {
  topic: string
  cells: number
  cutoff_cell_v: number
  resume_cell_v: number
  stale_s: number
}

/** Battery topic payload: serialized BatteryState plus the backend guard state. */
export interface BatteryPayload {
  voltage: number
  cells: number
  cell_voltage: number
  cutoff: boolean
  cutoff_v?: number
  resume_v?: number
  stamp?: number
}

export type BatteryLevel = 'ok' | 'low' | 'cutoff' | 'unknown'
export type BatteryColor = 'success' | 'warning' | 'error' | 'default'

export interface BatteryStatus {
  level: BatteryLevel
  color: BatteryColor
  label: string
  perCellLabel: string | null
  cellVoltage: number | null
}

const NO_DATA: BatteryStatus = { level: 'unknown', color: 'default', label: '--', perCellLabel: null, cellVoltage: null }

const LEVEL_COLORS: Record<Exclude<BatteryLevel, 'unknown'>, BatteryColor> = {
  ok: 'success',
  low: 'warning',
  cutoff: 'error',
}

/**
 * Level and display strings of the battery chip.
 * @param payload latest battery payload (undefined/null when none received)
 * @param cfg battery config from /api/config (null when battery features are off)
 * @param nowMs current time in ms
 * @param receivedAtMs time in ms the payload arrived, or null if never
 * @returns unknown (grey, "--") without data or when older than stale_s; red in cut-off (backend flag or voltage
 *   below cells x cutoff_cell_v); amber up to cells x resume_cell_v; otherwise green
 */
export function batteryStatus(
  payload: BatteryPayload | null | undefined,
  cfg: BatteryConfig | null | undefined,
  nowMs: number,
  receivedAtMs: number | null,
): BatteryStatus {
  if (!payload || !cfg || receivedAtMs === null || nowMs - receivedAtMs > cfg.stale_s * 1000) return NO_DATA
  const cellV = payload.voltage / cfg.cells
  let level: Exclude<BatteryLevel, 'unknown'>
  if (payload.cutoff || cellV < cfg.cutoff_cell_v) level = 'cutoff'
  else if (cellV <= cfg.resume_cell_v) level = 'low'
  else level = 'ok'
  return {
    level,
    color: LEVEL_COLORS[level],
    label: `${payload.voltage.toFixed(1)} V`,
    perCellLabel: `${cellV.toFixed(2)} V/cell`,
    cellVoltage: cellV,
  }
}

/**
 * Text of the red cut-off banner.
 * @returns the banner text in cut-off, otherwise null (no banner)
 */
export function cutoffBanner(status: BatteryStatus): string | null {
  if (status.level !== 'cutoff' || status.cellVoltage === null) return null
  return `Battery below cut-off (${status.cellVoltage.toFixed(2)} V/cell) - commands are disabled`
}
