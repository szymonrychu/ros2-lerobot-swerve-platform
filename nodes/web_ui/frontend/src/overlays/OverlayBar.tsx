import { useState } from 'react'
import Box from '@mui/material/Box'
import Chip from '@mui/material/Chip'
import IconButton from '@mui/material/IconButton'
import Stack from '@mui/material/Stack'
import Typography from '@mui/material/Typography'
import useMediaQuery from '@mui/material/useMediaQuery'
import { useTheme } from '@mui/material/styles'
import ExpandLessIcon from '@mui/icons-material/ExpandLess'
import ExpandMoreIcon from '@mui/icons-material/ExpandMore'
import { OverlayItem } from '../types'
import { extractField } from '../utils/fieldExtract'
import { RobotIcon } from '../components/RobotIcon'
import { BaseJointMessage } from '../map3d/baseJoints'
import { MONO_FONT } from '../theme'

interface Props {
  overlays: OverlayItem[]
  topicData: Record<string, unknown>
  /** Topic of the swerve JointState driving the robot icon. */
  jointStatesTopic?: string
  connected: boolean
}

/**
 * Status bar under the active tab: configured overlay values, swerve wheel yaw and link state.
 * Items wrap onto further lines when space runs out; on phones (below 'sm') the values collapse
 * behind a toggle so the bar takes one line and leaves the height to the tab.
 */
export function OverlayBar({ overlays, topicData, jointStatesTopic, connected }: Props) {
  const theme = useTheme()
  const narrow = useMediaQuery(theme.breakpoints.down('sm'))
  const [expanded, setExpanded] = useState(false)
  const showValues = !narrow || expanded

  return (
    <Box
      component="footer"
      sx={{
        flexShrink: 0,
        display: 'flex',
        alignItems: 'center',
        gap: 1,
        px: { xs: 1, sm: 1.5 },
        py: 0.5,
        bgcolor: 'background.paper',
        borderTop: 1,
        borderColor: 'divider',
      }}
    >
      <Stack
        direction="row"
        useFlexGap
        sx={{ flex: 1, minWidth: 0, flexWrap: 'wrap', alignItems: 'center', columnGap: { xs: 2, sm: 3 }, rowGap: 0.5 }}
      >
        {showValues &&
          overlays.map((item) => {
            const data = topicData[item.topic] as Record<string, unknown> | undefined
            const value = data ? extractField(data, item.field) : null
            const formatted =
              value !== null && value !== undefined
                ? item.format
                  ? value.toFixed(parseInt(item.format.replace('.', '').replace('f', '')))
                  : String(value)
                : '—'
            return (
              <Box key={`${item.topic}:${item.field}`} sx={{ display: 'flex', alignItems: 'baseline', gap: 0.75 }}>
                <Typography variant="overline" color="text.secondary" sx={{ lineHeight: 1.6 }}>
                  {item.label}
                </Typography>
                <Typography variant="body2" sx={{ fontFamily: MONO_FONT, fontWeight: 700 }}>
                  {formatted}
                </Typography>
                {item.unit && (
                  <Typography variant="caption" color="text.secondary">
                    {item.unit}
                  </Typography>
                )}
              </Box>
            )
          })}
        <RobotIcon joints={jointStatesTopic ? (topicData[jointStatesTopic] as BaseJointMessage | undefined) : undefined} />
      </Stack>
      <Chip
        size="small"
        variant="outlined"
        color={connected ? 'success' : 'default'}
        label={connected ? '● live' : '○ connecting…'}
        sx={{ flexShrink: 0 }}
      />
      {narrow && overlays.length > 0 && (
        <IconButton
          size="small"
          aria-label={expanded ? 'Hide status values' : 'Show status values'}
          aria-expanded={expanded}
          onClick={() => setExpanded((e) => !e)}
        >
          {expanded ? <ExpandMoreIcon /> : <ExpandLessIcon />}
        </IconButton>
      )}
    </Box>
  )
}
