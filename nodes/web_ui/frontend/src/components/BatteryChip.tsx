import Chip from '@mui/material/Chip'
import Tooltip from '@mui/material/Tooltip'
import BatteryAlertIcon from '@mui/icons-material/BatteryAlert'
import BatteryFullIcon from '@mui/icons-material/BatteryFull'
import BatteryUnknownIcon from '@mui/icons-material/BatteryUnknown'
import type { BatteryStatus } from '../battery/batteryStatus'

const TOOLTIPS = {
  unknown: 'Battery voltage unknown (no recent reading)',
  ok: 'Battery',
  low: 'Battery low',
  cutoff: 'Battery below cut-off - commands are disabled',
} as const

/** AppBar chip with the pack voltage; the per-cell voltage is in the tooltip. */
export function BatteryChip({ status }: { status: BatteryStatus }) {
  const icon =
    status.level === 'unknown' ? <BatteryUnknownIcon /> : status.level === 'cutoff' ? <BatteryAlertIcon /> : <BatteryFullIcon />
  const title = status.perCellLabel ? `${TOOLTIPS[status.level]}: ${status.label} (${status.perCellLabel})` : TOOLTIPS[status.level]
  return (
    <Tooltip title={title}>
      <Chip
        icon={icon}
        label={status.label}
        color={status.color}
        size="small"
        aria-label={title}
        sx={{ flexShrink: 0, fontWeight: 600, '& .MuiChip-label': { px: { xs: 0.75, sm: 1 } } }}
      />
    </Tooltip>
  )
}
