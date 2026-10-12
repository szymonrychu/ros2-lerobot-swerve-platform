import { useState } from 'react'
import Box from '@mui/material/Box'
import IconButton from '@mui/material/IconButton'
import Stack from '@mui/material/Stack'
import ExpandLessIcon from '@mui/icons-material/ExpandLess'
import ExpandMoreIcon from '@mui/icons-material/ExpandMore'
import { OverlayItem } from '../types'
import { extractField } from '../utils/fieldExtract'
import { RobotIcon } from '../components/RobotIcon'
import { BaseJointMessage } from '../map3d/baseJoints'
import { useLayout } from '../layout/useLayout'
import { StatusItem } from '../components/StatusItem'

interface Props {
  overlays: OverlayItem[]
  topicData: Record<string, unknown>
  /** Topic of the swerve JointState driving the robot icon. */
  jointStatesTopic?: string
  connected: boolean
}

/**
 * Status bar under the active tab: configured overlay values, swerve wheel yaw and link state.
 * Items wrap onto further lines when space runs out; on phones (layout class 'phone') the values collapse
 * behind a toggle so the bar takes one line and leaves the height to the tab.
 */
export function OverlayBar({ overlays, topicData, jointStatesTopic, connected }: Props) {
  const narrow = useLayout() === 'phone'
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
        px: { xs: 1, sm: 2 },
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
                : '--'
            return <StatusItem key={`${item.topic}:${item.field}`} label={item.label} value={formatted} unit={item.unit} />
          })}
        <RobotIcon joints={jointStatesTopic ? (topicData[jointStatesTopic] as BaseJointMessage | undefined) : undefined} />
      </Stack>
      <StatusItem label="" value={connected ? 'Live' : 'Connecting'} />
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
