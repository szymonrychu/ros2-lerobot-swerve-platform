import { StatusItem, type StatusState } from './StatusItem'
import type { BatteryStatus } from '../battery/batteryStatus'

const TOOLTIPS = {
  unknown: 'Battery voltage unknown (no recent reading)',
  ok: 'Battery',
  low: 'Battery low',
  cutoff: 'Battery below cut-off - commands are disabled',
} as const

const STATES: Record<BatteryStatus['level'], StatusState> = {
  unknown: 'unknown',
  ok: 'ok',
  low: 'warn',
  cutoff: 'error',
}

/** Status bar reading with the pack voltage; the per-cell voltage is in the tooltip. */
export function BatteryChip({ status }: { status: BatteryStatus }) {
  const title = status.perCellLabel ? `${TOOLTIPS[status.level]}: ${status.label} (${status.perCellLabel})` : TOOLTIPS[status.level]
  return <StatusItem label="Battery" value={status.label} state={STATES[status.level]} tooltip={title} />
}
