/**
 * Layout class of the viewport, built on the theme breakpoints (sm = 600, lg = 1200):
 * width < 600 is 'phone', 600..1199 is 'tablet', >= 1200 is 'desktop'.
 */
import { useMediaQuery, useTheme } from '@mui/material'

export type LayoutClass = 'phone' | 'tablet' | 'desktop'

/**
 * @returns the current layout class, updated when the viewport crosses a breakpoint.
 */
export function useLayout(): LayoutClass {
  const theme = useTheme()
  const isTabletUp = useMediaQuery(theme.breakpoints.up('sm'))
  const isDesktop = useMediaQuery(theme.breakpoints.up('lg'))
  if (isDesktop) return 'desktop'
  return isTabletUp ? 'tablet' : 'phone'
}
