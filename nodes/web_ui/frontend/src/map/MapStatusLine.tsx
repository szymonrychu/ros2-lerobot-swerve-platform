import { Children } from 'react'
import type { ReactNode } from 'react'
import Box from '@mui/material/Box'
import { PANEL_BG, TOKENS } from '../theme'
import type { MapStatus } from '../types'

/** Message map_manager gives when a moved robot could not be relocalised and the map was dropped. */
const DISCARDED_MESSAGE = 'map discarded'

/**
 * The one-line text for the active map and its localisation state.
 *
 * @param status - latest map status, undefined before the first one arrives.
 * @returns the text, or null when there is nothing to say yet.
 */
export function mapStatusText(status: MapStatus | undefined): string | null {
  if (!status) return null
  const { state, message } = status.localisation
  const name = status.active_map
  if (state === 'not_localised' && message.includes(DISCARDED_MESSAGE)) {
    return 'Map cleared after the robot was moved; load a saved map'
  }
  if (state === 'localised') return `Localised on ${name ?? 'the map'}`
  if (state === 'not_localised' || state === 'ambiguous') {
    return `Not localised${name ? ` on ${name}` : ''}${message ? `: ${message}` : ''}`
  }
  if (state === 'loading') return name ? `Loading ${name}` : 'Loading map'
  return name ? `Map ${name}` : null
}

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
