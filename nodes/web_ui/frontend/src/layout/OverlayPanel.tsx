/**
 * Floating panel drawn over the map or a 3D view. Phone: bottom sheet (max 60dvh).
 * Flat: opaque background, 1 px border, radius 2, no shadow or blur.
 * Tablet and desktop: floating card of width clamp(280px, 30vw, 380px) anchored left, right or bottom.
 */
import type { CSSProperties, ReactNode } from 'react'
import { Box, IconButton, Typography, useMediaQuery } from '@mui/material'
import CloseIcon from '@mui/icons-material/Close'
import { PANEL_BG, TOKENS, TOUCH_TARGET_PX } from '../theme'
import { useLayout } from './useLayout'

const CARD_WIDTH = 'clamp(280px, 30vw, 380px)'
const SHEET_MAX_HEIGHT = '60dvh'
const CARD_MARGIN = '12px'
const Z_INDEX = 5
const TOUCH_STYLE: CSSProperties = { minWidth: TOUCH_TARGET_PX, minHeight: TOUCH_TARGET_PX }

export interface OverlayPanelProps {
  /** Heading shown in the panel header. */
  title: string
  /** Closed panels render nothing. */
  open: boolean
  /** Edge the floating card sticks to on tablet and desktop (phone always uses the bottom edge). */
  anchor: 'left' | 'right' | 'bottom'
  /** When set, a close button is shown and calls this. */
  onClose?: () => void
  /** Tablet and desktop only: lay the card out in the normal flow of its parent (a docked column) instead of floating. Phone stays a bottom sheet. */
  docked?: boolean
  /** Controls shown on the right of the header, before the close button. */
  actions?: ReactNode
  children?: ReactNode
}

/**
 * @param props see OverlayPanelProps.
 * @returns the panel, or null when closed.
 */
export function OverlayPanel({ title, open, anchor, onClose, docked = false, actions, children }: OverlayPanelProps) {
  const layout = useLayout()
  const coarse = useMediaQuery('(pointer: coarse)')
  if (!open) return null

  const phone = layout === 'phone'
  let position: CSSProperties
  if (phone) {
    position = { left: 0, right: 0, bottom: 0, maxHeight: SHEET_MAX_HEIGHT, borderRadius: '0' }
  } else if (docked) {
    position = { position: 'relative', width: '100%', maxHeight: '75dvh' }
  } else if (anchor === 'bottom') {
    position = { bottom: CARD_MARGIN, left: '50%', transform: 'translateX(-50%)', width: CARD_WIDTH, maxHeight: '60vh' }
  } else {
    position = { top: CARD_MARGIN, [anchor]: CARD_MARGIN, width: CARD_WIDTH, maxHeight: `calc(100% - 2 * ${CARD_MARGIN})` }
  }

  return (
    <Box
      data-testid="overlay-panel"
      data-layout={layout}
      onPointerDown={(e) => e.stopPropagation()}
      style={{ position: 'absolute', zIndex: Z_INDEX, flexShrink: 0, background: PANEL_BG, ...position }}
      sx={{ display: 'flex', flexDirection: 'column', overflow: 'hidden', border: 1, borderColor: 'divider', borderRadius: `${TOKENS.radius}px` }}
    >
      <Box
        data-testid="overlay-panel-header"
        style={{ minHeight: coarse ? TOUCH_TARGET_PX : TOKENS.panelHeaderPx }}
        sx={{ display: 'flex', alignItems: 'center', gap: 1, px: 1, flexShrink: 0, borderBottom: 1, borderColor: 'divider' }}
      >
        <Typography noWrap sx={{ flex: 1, fontSize: TOKENS.fontSizes.sm, fontWeight: TOKENS.fontWeights.medium, color: 'text.secondary' }}>
          {title}
        </Typography>
        {actions}
        {onClose && (
          <IconButton aria-label="Close" size="small" onClick={onClose} style={coarse ? TOUCH_STYLE : undefined}>
            <CloseIcon fontSize="small" />
          </IconButton>
        )}
      </Box>
      <Box sx={{ overflow: 'auto', p: 1 }}>{children}</Box>
    </Box>
  )
}
