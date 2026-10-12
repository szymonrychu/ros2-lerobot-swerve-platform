import Box from '@mui/material/Box'
import { StatusItem, type StatusState } from './StatusItem'
import type { GpsChipStatus } from '../gps/gpsStatus'

const STATES: Record<GpsChipStatus['color'], StatusState> = {
  error: 'error',
  warning: 'warn',
  default: 'unknown',
  info: 'ok',
  success: 'ok',
}

/** Status bar reading with a GPS fix summary; the details are in the tooltip, and only the short label shows on phones. */
export function GpsChip({ status }: { status: GpsChipStatus; role: 'robot' | 'base' }) {
  return (
    <StatusItem
      label=""
      value={
        <>
          <Box component="span" sx={{ display: { xs: 'inline', sm: 'none' } }}>
            {status.shortLabel}
          </Box>
          <Box component="span" sx={{ display: { xs: 'none', sm: 'inline' } }}>
            {status.label}
          </Box>
        </>
      }
      state={STATES[status.color]}
      tooltip={<span style={{ whiteSpace: 'pre-line' }}>{status.tooltip}</span>}
      ariaLabel={status.tooltip.replace(/\n/g, ', ')}
    />
  )
}
