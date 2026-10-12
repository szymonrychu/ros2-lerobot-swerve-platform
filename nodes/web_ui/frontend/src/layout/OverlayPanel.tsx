/**
 * Floating panel drawn over the map or a 3D view. Phone: bottom sheet with a drag handle (max 60dvh).
 * Tablet and desktop: floating card of width clamp(280px, 30vw, 380px) anchored left, right or bottom.
 */
import type { CSSProperties, ReactNode } from 'react'
import { Box, IconButton, Typography } from '@mui/material'
import CloseIcon from '@mui/icons-material/Close'
import { PANEL_BG, TOUCH_TARGET_PX } from '../theme'
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
  children?: ReactNode
}

/**
 * @param props see OverlayPanelProps.
 * @returns the panel, or null when closed.
 */
export function OverlayPanel({ title, open, anchor, onClose, docked = false, children }: OverlayPanelProps) {
  const layout = useLayout()
  if (!open) return null

  const phone = layout === 'phone'
  let position: CSSProperties
  if (phone) {
    position = { left: 0, right: 0, bottom: 0, maxHeight: SHEET_MAX_HEIGHT, borderRadius: '16px 16px 0 0' }
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
      sx={{ display: 'flex', flexDirection: 'column', overflow: 'hidden', border: 1, borderColor: 'divider', borderRadius: 2, backdropFilter: 'blur(6px)' }}
    >
      {phone && (
        <Box
          data-testid="overlay-panel-handle"
          style={{ minHeight: TOUCH_TARGET_PX }}
          sx={{ display: 'flex', alignItems: 'center', justifyContent: 'center', flexShrink: 0, touchAction: 'none' }}
        >
          <Box sx={{ width: 40, height: 4, borderRadius: 2, bgcolor: 'text.secondary', opacity: 0.6 }} />
        </Box>
      )}
      <Box sx={{ display: 'flex', alignItems: 'center', px: 1.5, flexShrink: 0 }}>
        <Typography variant="overline" sx={{ flex: 1 }}>{title}</Typography>
        {onClose && (
          <IconButton aria-label="Close" size="small" onClick={onClose} style={TOUCH_STYLE}>
            <CloseIcon fontSize="small" />
          </IconButton>
        )}
      </Box>
      <Box sx={{ overflow: 'auto', px: 1.5, pb: 1.5 }}>{children}</Box>
    </Box>
  )
}
