import { describe, expect, it } from 'vitest'
import { CANVAS_BG, CONTROL_PX, MONO_FONT, PANEL_BG, TOKENS, TOUCH_TARGET_PX, theme } from './theme'

const channels = (hex: string): number[] => [1, 3, 5].map((i) => parseInt(hex.slice(i, i + 2), 16))
// eslint-disable-next-line @typescript-eslint/no-explicit-any
const comp = theme.components as any
const COARSE = '@media (pointer: coarse)'

describe('TOKENS', () => {
  it('has the surface, border, text and state values', () => {
    expect(TOKENS).toMatchObject({
      bgCanvas: '#0f1013', bgPanel: '#16181d', bgRaised: '#1d2026',
      border: '#2a2e36', borderStrong: '#3a3f49',
      textPrimary: '#d6d9de', textSecondary: '#8d939c', textDisabled: '#5a6069',
      error: '#e5484d', warning: '#e2a336',
      radius: 2, borderWidth: 1, panelHeaderPx: 32, controlPx: 32, touchTargetPx: 44,
      fontSizes: { xs: 11, sm: 12, md: 13, lg: 14, xl: 16 },
      fontWeights: { regular: 400, medium: 500 },
    })
    expect(TOKENS.fontWeights).toEqual({ regular: 400, medium: 500 })
    expect('ok' in TOKENS).toBe(false)
  })

  it('accent is a neutral light grey and accentText is dark', () => {
    const [r, g, b] = channels(TOKENS.accent)
    expect(Math.max(r, g, b) - Math.min(r, g, b)).toBeLessThanOrEqual(8)
    expect(Math.min(r, g, b)).toBeGreaterThanOrEqual(0xd0)
    expect(Math.max(...channels(TOKENS.accentText))).toBeLessThanOrEqual(0x30)
  })
})

describe('MUI palette', () => {
  const p = theme.palette
  it('maps tokens', () => {
    expect(p.mode).toBe('dark')
    expect(p.primary.main).toBe(TOKENS.accent)
    expect(p.secondary.main).toBe(TOKENS.textSecondary)
    expect(p.info.main).toBe(TOKENS.textSecondary)
    expect(p.success.main).toBe(TOKENS.textPrimary)
    expect(p.error.main).toBe(TOKENS.error)
    expect(p.warning.main).toBe(TOKENS.warning)
    expect(p.background.default).toBe(TOKENS.bgCanvas)
    expect(p.background.paper).toBe(TOKENS.bgPanel)
    expect(p.divider).toBe(TOKENS.border)
    expect(p.text.primary).toBe(TOKENS.textPrimary)
    expect(p.text.secondary).toBe(TOKENS.textSecondary)
    expect(p.text.disabled).toBe(TOKENS.textDisabled)
    expect(theme.shape.borderRadius).toBe(2)
  })
})

describe('typography', () => {
  it('is compact, system font, weights 400/500', () => {
    const t = theme.typography
    expect(t.fontSize).toBe(13)
    expect(t.fontFamily).not.toMatch(/https?:|@font-face/)
    expect(MONO_FONT).toContain('monospace')
    const variants = ['h1', 'h2', 'h3', 'h4', 'h5', 'h6', 'subtitle1', 'subtitle2', 'body1', 'body2', 'button', 'caption', 'overline'] as const
    for (const v of variants) expect([400, 500]).toContain(t[v].fontWeight)
    expect(t.overline.textTransform).toBe('none')
    expect(t.overline.letterSpacing).toBe(0)
    expect(t.button.textTransform).toBe('none')
  })
})

describe('component overrides', () => {
  it('Chip, Alert, Paper', () => {
    expect(comp.MuiChip.defaultProps).toMatchObject({ variant: 'outlined', size: 'small' })
    expect(comp.MuiChip.styleOverrides.root.borderRadius).toBe(2)
    expect(comp.MuiAlert.defaultProps.variant).toBe('outlined')
    expect(comp.MuiAlert.styleOverrides.root).toMatchObject({ borderRadius: 2, borderLeftWidth: 3 })
    expect(comp.MuiPaper.defaultProps.elevation).toBe(0)
    expect(comp.MuiPaper.styleOverrides.root).toMatchObject({
      backgroundImage: 'none',
      border: `1px solid ${TOKENS.border}`,
    })
  })

  it('Button and IconButton are 44 px on coarse pointers and 32 px otherwise', () => {
    expect(comp.MuiButton.defaultProps.disableElevation).toBe(true)
    for (const name of ['MuiButton', 'MuiIconButton']) {
      const root = comp[name].styleOverrides.root
      expect(root.minHeight).toBe(CONTROL_PX)
      expect(root.minWidth).toBe(CONTROL_PX)
      expect(root[COARSE]).toMatchObject({ minHeight: TOUCH_TARGET_PX, minWidth: TOUCH_TARGET_PX })
    }
  })

  it('Tooltip, Tab, LinearProgress', () => {
    expect(comp.MuiTooltip.styleOverrides.tooltip).toMatchObject({ fontSize: 11, backgroundColor: TOKENS.bgRaised })
    expect(comp.MuiTooltip.styleOverrides.tooltip.border).toContain(TOKENS.border)
    expect(comp.MuiTab.styleOverrides.root).toMatchObject({ minHeight: 40, fontWeight: 500 })
    expect(comp.MuiTabs.styleOverrides.indicator.backgroundColor).toBe(TOKENS.accent)
    expect(comp.MuiLinearProgress.styleOverrides.root).toMatchObject({ height: 2, borderRadius: 0 })
  })

  it('CssBaseline sets tabular numbers and canvas background', () => {
    const css = comp.MuiCssBaseline.styleOverrides
    expect(css.body.fontVariantNumeric).toBe('tabular-nums')
    expect(css.body.backgroundColor).toBe(TOKENS.bgCanvas)
  })
})

describe('exported constants', () => {
  it('are opaque token aliases', () => {
    expect(CANVAS_BG).toBe(TOKENS.bgCanvas)
    expect(PANEL_BG).toBe(TOKENS.bgPanel)
    expect(TOUCH_TARGET_PX).toBe(44)
    expect(CONTROL_PX).toBe(32)
  })
})
