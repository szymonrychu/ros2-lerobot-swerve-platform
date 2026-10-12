/**
 * The single Material Design (MUI) theme of the dashboard: neutral dark, flat, colour only for state.
 *
 * Fonts come from the system stacks (no web-font download: the robot UI must work offline and under CSP).
 */
import { createTheme } from '@mui/material/styles'

export const TOKENS = {
  // surfaces
  bgCanvas: '#0f1013',
  bgPanel: '#16181d',
  bgRaised: '#1d2026',
  border: '#2a2e36',
  borderStrong: '#3a3f49',
  // text
  textPrimary: '#d6d9de',
  textSecondary: '#8d939c',
  textDisabled: '#5a6069',
  // the one accent: neutral light grey, no hue
  accent: '#e8eaed',
  accentText: '#0f1013',
  // state only
  error: '#e5484d',
  warning: '#e2a336',
  // shape and density
  radius: 2,
  borderWidth: 1,
  panelHeaderPx: 32,
  controlPx: 32,
  touchTargetPx: 44,
  // type
  fontSizes: { xs: 11, sm: 12, md: 13, lg: 14, xl: 16 },
  fontWeights: { regular: 400, medium: 500 },
} as const

export const MONO_FONT = '"SFMono-Regular", Menlo, Consolas, "Liberation Mono", monospace'
export const SANS_FONT = 'system-ui, -apple-system, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif'
/** Minimum size of an interactive target on coarse pointers, per Material/WCAG touch guidance. */
export const TOUCH_TARGET_PX = TOKENS.touchTargetPx
/** Size of a dense control on fine pointers. */
export const CONTROL_PX = TOKENS.controlPx
/** Background behind canvases, 3D views and plots. */
export const CANVAS_BG = TOKENS.bgCanvas
/** Opaque background of panels, including those drawn over the map or 3D view. */
export const PANEL_BG = TOKENS.bgPanel

const { regular, medium } = TOKENS.fontWeights
const { xs, sm, md, lg, xl } = TOKENS.fontSizes
const COARSE = '@media (pointer: coarse)'
const sized = (px: number, fontWeight: number) => ({ fontSize: `${px / 16}rem`, fontWeight, letterSpacing: 0 })
const BORDER = `${TOKENS.borderWidth}px solid ${TOKENS.border}`
const touchSized = {
  minHeight: CONTROL_PX,
  minWidth: CONTROL_PX,
  [COARSE]: { minHeight: TOUCH_TARGET_PX, minWidth: TOUCH_TARGET_PX },
}

export const theme = createTheme({
  palette: {
    mode: 'dark',
    primary: { main: TOKENS.accent, contrastText: TOKENS.accentText },
    secondary: { main: TOKENS.textSecondary },
    info: { main: TOKENS.textSecondary },
    success: { main: TOKENS.textPrimary },
    error: { main: TOKENS.error },
    warning: { main: TOKENS.warning },
    background: { default: TOKENS.bgCanvas, paper: TOKENS.bgPanel },
    divider: TOKENS.border,
    text: { primary: TOKENS.textPrimary, secondary: TOKENS.textSecondary, disabled: TOKENS.textDisabled },
  },
  shape: { borderRadius: TOKENS.radius },
  typography: {
    fontFamily: SANS_FONT,
    fontSize: md,
    fontWeightLight: regular,
    fontWeightRegular: regular,
    fontWeightMedium: medium,
    fontWeightBold: medium,
    h1: sized(xl, medium),
    h2: sized(xl, medium),
    h3: sized(xl, medium),
    h4: sized(xl, medium),
    h5: sized(lg, medium),
    h6: sized(lg, medium),
    subtitle1: sized(lg, regular),
    subtitle2: sized(md, medium),
    body1: sized(md, regular),
    body2: sized(sm, regular),
    caption: sized(xs, regular),
    overline: { ...sized(sm, medium), textTransform: 'none' },
    button: { ...sized(md, medium), textTransform: 'none' },
  },
  components: {
    MuiCssBaseline: {
      styleOverrides: {
        body: { fontVariantNumeric: 'tabular-nums', backgroundColor: TOKENS.bgCanvas },
      },
    },
    MuiButton: {
      defaultProps: { disableElevation: true },
      styleOverrides: { root: { ...touchSized, textTransform: 'none', fontWeight: medium } },
    },
    MuiIconButton: {
      styleOverrides: { root: touchSized },
    },
    MuiChip: {
      defaultProps: { variant: 'outlined', size: 'small' },
      styleOverrides: { root: { borderRadius: TOKENS.radius, fontSize: xs } },
    },
    MuiAlert: {
      defaultProps: { variant: 'outlined' },
      styleOverrides: { root: { borderRadius: TOKENS.radius, borderLeftWidth: 3 } },
    },
    MuiTooltip: {
      styleOverrides: {
        tooltip: { fontSize: xs, backgroundColor: TOKENS.bgRaised, color: TOKENS.textPrimary, border: BORDER },
      },
    },
    MuiTabs: {
      styleOverrides: { indicator: { backgroundColor: TOKENS.accent } },
    },
    MuiTab: {
      styleOverrides: { root: { minHeight: 40, textTransform: 'none', fontWeight: medium } },
    },
    MuiLinearProgress: {
      styleOverrides: { root: { height: 2, borderRadius: 0 } },
    },
    MuiAppBar: {
      defaultProps: { elevation: 0, color: 'default' },
      styleOverrides: { root: { borderBottom: BORDER } },
    },
    MuiPaper: {
      defaultProps: { elevation: 0 },
      styleOverrides: { root: { backgroundImage: 'none', border: BORDER } },
    },
  },
})
