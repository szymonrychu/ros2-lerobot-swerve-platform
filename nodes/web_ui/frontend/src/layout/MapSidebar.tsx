/**
 * Docked side column next to the map. Tablet and desktop: an opaque right column that takes its own width,
 * so the map beside it shrinks instead of being covered; OverlayPanels inside it render in flow as stacked
 * sections. Phone: the children render unchanged, so the panels stay bottom sheets over the map.
 */
import { createContext, useContext } from 'react'
import type { ReactNode } from 'react'
import Box from '@mui/material/Box'
import { PANEL_BG, TOKENS } from '../theme'
import { useLayout } from './useLayout'

/** Width of the docked column (same clamp as a floating panel card). */
export const SIDEBAR_WIDTH = 'clamp(280px, 30vw, 380px)'

const DockedContext = createContext(false)

/**
 * @returns true when the caller renders inside a docked MapSidebar column (tablet and desktop only).
 */
export function useDockedInSidebar(): boolean {
  return useContext(DockedContext)
}

export interface MapSidebarProps {
  /** Panels shown in the column, top to bottom. */
  children?: ReactNode
}

/**
 * @param props - see MapSidebarProps.
 * @returns the docked column on tablet and desktop, the bare children on phone.
 */
export function MapSidebar({ children }: MapSidebarProps) {
  const layout = useLayout()
  if (layout === 'phone') return <>{children}</>
  return (
    <Box
      component="aside"
      aria-label="Map panels"
      data-testid="map-sidebar"
      // A press in the column must never start a map gesture (goal, POI add, grasp pick).
      onPointerDown={(e) => e.stopPropagation()}
      style={{
        width: SIDEBAR_WIDTH,
        height: '100%',
        flexShrink: 0,
        overflowY: 'auto',
        background: PANEL_BG,
        borderLeft: `${TOKENS.borderWidth}px solid ${TOKENS.border}`,
        boxSizing: 'border-box',
      }}
      sx={{ display: 'flex', flexDirection: 'column' }}
    >
      <DockedContext.Provider value>{children}</DockedContext.Provider>
    </Box>
  )
}
