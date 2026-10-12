import type { ReactNode } from 'react'
import Box from '@mui/material/Box'
import Tooltip from '@mui/material/Tooltip'
import { MONO_FONT, TOKENS } from '../theme'

export type StatusState = 'ok' | 'warn' | 'error' | 'unknown'

/** Pixel size of the square state mark. */
const MARK_PX = 6
const STATE_COLOUR = { warn: TOKENS.warning, error: TOKENS.error } as const

interface Props {
  /** Small caption left of the value; omitted when empty. */
  label: string
  value: ReactNode
  unit?: string
  /** warn and error colour the value and add a square mark; ok and unknown stay neutral (never green). */
  state?: StatusState
  /** Details shown on hover and used as the accessible name. */
  tooltip?: ReactNode
  /** Accessible name when the tooltip is not plain text. */
  ariaLabel?: string
}

/**
 * One quiet instrument reading: label, monospace value and unit; only a warn or error state draws the eye.
 *
 * @param props - label, value, optional unit, state and tooltip
 * @returns the reading as an inline row
 */
export function StatusItem({ label, value, unit, state = 'ok', tooltip, ariaLabel }: Props) {
  const colour = state === 'warn' || state === 'error' ? STATE_COLOUR[state] : null
  const item = (
    <Box
      aria-label={ariaLabel ?? (typeof tooltip === 'string' ? tooltip : undefined)}
      sx={{
        display: 'inline-flex',
        alignItems: 'baseline',
        gap: 0.5,
        flexShrink: 0,
        whiteSpace: 'nowrap',
      }}
    >
      {colour && (
        <Box
          data-testid="status-mark"
          sx={{
            width: MARK_PX,
            height: MARK_PX,
            bgcolor: colour,
            alignSelf: 'center',
            flexShrink: 0,
          }}
        />
      )}
      {label && (
        <Box component="span" sx={{ fontSize: TOKENS.fontSizes.xs, color: 'text.secondary' }}>
          {label}
        </Box>
      )}
      <Box
        component="span"
        sx={{
          fontSize: TOKENS.fontSizes.md,
          fontFamily: MONO_FONT,
          fontVariantNumeric: 'tabular-nums',
          color: colour ?? 'text.primary',
        }}
      >
        {value}
      </Box>
      {unit && (
        <Box component="span" sx={{ fontSize: TOKENS.fontSizes.xs, color: 'text.secondary' }}>
          {unit}
        </Box>
      )}
    </Box>
  )
  return tooltip ? <Tooltip title={tooltip}>{item}</Tooltip> : item
}
