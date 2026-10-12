import { Children } from 'react'
import type { ReactNode } from 'react'
import Box from '@mui/material/Box'
import { PANEL_BG, TOKENS } from '../theme'

interface Props {
  /** StatusItems or plain text, shown left to right on one line. */
  items: readonly ReactNode[]
}

/**
 * One quiet line in the bottom-left corner of the map: the mode hint, the robot pose.
 *
 * @param props - items to show
 * @returns the line, or null when there is nothing to show
 */
export function MapStatusLine({ items }: Props) {
  if (items.length === 0) return null
  return (
    <Box
      role="status"
      aria-live="polite"
      data-testid="map-status-line"
      sx={{
        position: 'absolute',
        left: 8,
        bottom: 8,
        maxWidth: 'calc(100% - 16px)',
        display: 'flex',
        alignItems: 'baseline',
        gap: 1,
        px: 1,
        py: 0.5,
        overflow: 'hidden',
        whiteSpace: 'nowrap',
        bgcolor: PANEL_BG,
        border: `${TOKENS.borderWidth}px solid ${TOKENS.border}`,
        borderRadius: `${TOKENS.radius}px`,
        fontSize: TOKENS.fontSizes.sm,
        color: 'text.secondary',
        pointerEvents: 'none',
        zIndex: 2,
      }}
    >
      {Children.toArray(items as ReactNode[])}
    </Box>
  )
}
