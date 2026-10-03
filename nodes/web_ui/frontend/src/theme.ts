/**
 * The single Material Design (MUI) theme of the dashboard: dark, high contrast, touch-sized controls.
 *
 * Fonts come from the system stack (no web-font download: the robot UI must work offline and under CSP).
 */
import { createTheme } from '@mui/material/styles'

export const MONO_FONT = '"JetBrains Mono", "SFMono-Regular", Menlo, Consolas, "Liberation Mono", monospace'
export const SANS_FONT = 'Roboto, "Segoe UI", system-ui, -apple-system, "Helvetica Neue", Arial, sans-serif'
/** Minimum size of an interactive target, per Material/WCAG touch guidance. */
export const TOUCH_TARGET_PX = 44
/** Background behind canvases, 3D views and plots. */
export const CANVAS_BG = '#0b0d10'

export const theme = createTheme({
  palette: {
    mode: 'dark',
    primary: { main: '#4fc3f7' },
    secondary: { main: '#ffca28' },
    error: { main: '#ef5350' },
    warning: { main: '#ffa726' },
    success: { main: '#66bb6a' },
    background: { default: '#0e1116', paper: '#161b22' },
    divider: 'rgba(255, 255, 255, 0.12)',
    text: { primary: '#e6edf3', secondary: '#9da7b3' },
  },
  shape: { borderRadius: 8 },
  typography: {
    fontFamily: SANS_FONT,
    overline: { letterSpacing: '0.1em', fontWeight: 600 },
  },
  components: {
    MuiButton: {
      defaultProps: { disableElevation: true },
      styleOverrides: { root: { minHeight: TOUCH_TARGET_PX, textTransform: 'none', fontWeight: 600 } },
    },
    MuiIconButton: {
      styleOverrides: { root: { minWidth: TOUCH_TARGET_PX, minHeight: TOUCH_TARGET_PX } },
    },
    MuiTab: {
      styleOverrides: { root: { minHeight: 48, textTransform: 'none', fontWeight: 600 } },
    },
    MuiAppBar: {
      defaultProps: { elevation: 0, color: 'default' },
      styleOverrides: { root: { borderBottom: '1px solid rgba(255, 255, 255, 0.12)' } },
    },
    MuiPaper: {
      styleOverrides: { root: { backgroundImage: 'none' } },
    },
  },
})
