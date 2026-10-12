import type { MouseEvent, ReactNode } from 'react'
import Box from '@mui/material/Box'
import IconButton from '@mui/material/IconButton'
import Tooltip from '@mui/material/Tooltip'
import { TOKENS } from '../theme'

interface ToolButtonProps {
  /** Tooltip text, also the accessible name. */
  label: string
  icon: ReactNode
  onClick: (event: MouseEvent<HTMLElement>) => void
  /** Toggle state; leave undefined for a plain action so no aria-pressed is set. */
  pressed?: boolean
  disabled?: boolean
  /** For a button that opens a menu: whether the menu is open. Sets aria-haspopup and aria-expanded. */
  expanded?: boolean
}

/**
 * Icon-only toolbar control: an IconButton in a Tooltip. A toggle that is on shows the accent icon colour on bgRaised.
 *
 * @param props - label (tooltip and aria-label), icon, onClick, optional pressed, disabled and expanded
 * @returns the button
 */
export function ToolButton({ label, icon, onClick, pressed, disabled = false, expanded }: ToolButtonProps) {
  return (
    <Tooltip title={label}>
      <span>
        <IconButton
          aria-label={label}
          aria-pressed={pressed}
          aria-haspopup={expanded === undefined ? undefined : 'menu'}
          aria-expanded={expanded}
          disabled={disabled}
          onClick={onClick}
          sx={{
            color: pressed ? TOKENS.accent : 'text.secondary',
            bgcolor: pressed ? TOKENS.bgRaised : 'transparent',
            '&:hover': { bgcolor: TOKENS.bgRaised, color: TOKENS.accent },
          }}
        >
          {icon}
        </IconButton>
      </span>
    </Tooltip>
  )
}

interface ToolGroupProps {
  /** Accessible name of the group. */
  label: string
  children: ReactNode
}

/**
 * Buttons that belong together, sharing one 1 px bordered strip.
 *
 * @param props - label and the ToolButtons
 * @returns the strip
 */
export function ToolGroup({ label, children }: ToolGroupProps) {
  return (
    <Box
      role="group"
      aria-label={label}
      sx={{
        display: 'inline-flex',
        flexShrink: 0,
        border: `${TOKENS.borderWidth}px solid ${TOKENS.border}`,
        borderRadius: `${TOKENS.radius}px`,
      }}
    >
      {children}
    </Box>
  )
}
