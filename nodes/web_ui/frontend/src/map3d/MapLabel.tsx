/** The one map label style: a flat instrument tag. Only a zone label is coloured, and only on its border. */
import type { ReactNode } from 'react'
import { SANS_FONT, TOKENS } from '../theme'
import { ZONE_COLORS } from './palette'

const LABEL_FONT_PX = 11
const LABEL_RADIUS_PX = 2

/**
 * A label for a map or scene marker, to place inside a drei Html.
 * @param children - label content.
 * @param zone - zone the label names; its border takes the palette colour of that zone.
 * @returns the label element.
 */
export function MapLabel({ children, zone, title }: { children: ReactNode; zone?: 'keepout' | 'slow'; title?: string }) {
  return (
    <div
      title={title}
      style={{
        transform: 'translateY(-14px)',
        whiteSpace: 'nowrap',
        fontFamily: SANS_FONT,
        fontSize: LABEL_FONT_PX,
        fontWeight: TOKENS.fontWeights.medium,
        color: TOKENS.textPrimary,
        background: TOKENS.bgPanel,
        border: `${TOKENS.borderWidth}px solid ${zone ? ZONE_COLORS[zone] : TOKENS.border}`,
        borderRadius: LABEL_RADIUS_PX,
        padding: '1px 5px',
      }}
    >
      {children}
    </div>
  )
}
