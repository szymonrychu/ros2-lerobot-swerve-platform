import Box from '@mui/material/Box'
import Chip from '@mui/material/Chip'
import Tooltip from '@mui/material/Tooltip'
import CellTowerIcon from '@mui/icons-material/CellTower'
import GpsFixedIcon from '@mui/icons-material/GpsFixed'
import GpsNotFixedIcon from '@mui/icons-material/GpsNotFixed'
import GpsOffIcon from '@mui/icons-material/GpsOff'
import { roverIcon, type GpsChipStatus } from '../gps/gpsStatus'

/** AppBar chip with a GPS fix summary; the details are in the tooltip, and only the short label shows on phones. */
export function GpsChip({ status, role }: { status: GpsChipStatus; role: 'robot' | 'base' }) {
  const kind = roverIcon(status.level)
  const icon =
    role === 'base' ? (
      <CellTowerIcon />
    ) : kind === 'off' ? (
      <GpsOffIcon />
    ) : kind === 'fixed' ? (
      <GpsFixedIcon />
    ) : (
      <GpsNotFixedIcon />
    )
  return (
    <Tooltip title={<span style={{ whiteSpace: 'pre-line' }}>{status.tooltip}</span>}>
      <Chip
        icon={icon}
        label={
          <>
            <Box component="span" sx={{ display: { xs: 'inline', sm: 'none' } }}>
              {status.shortLabel}
            </Box>
            <Box component="span" sx={{ display: { xs: 'none', sm: 'inline' } }}>
              {status.label}
            </Box>
          </>
        }
        color={status.color}
        size="small"
        aria-label={status.tooltip.replace(/\n/g, ', ')}
        sx={{
          flexShrink: 0,
          fontWeight: 600,
          maxWidth: { xs: '7.5rem', sm: 'none' },
          '& .MuiChip-label': { px: { xs: 0.75, sm: 1 } },
          '& .MuiChip-icon': { display: { xs: 'none', sm: 'inline-block' } },
        }}
      />
    </Tooltip>
  )
}
