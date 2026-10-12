import { describe, expect, it } from 'vitest'

/** Guard: the responsive overhaul stays done. Fixed pixel layout sizes must not creep back into the migrated files. */

/** Raw text of every non-test source file below src, keyed by its path relative to src (e.g. "tabs/MapNavTab.tsx"). */
const SOURCES: Record<string, string> = Object.fromEntries(
  Object.entries(import.meta.glob(['../**/*.ts', '../**/*.tsx', '!../**/*.test.ts', '!../**/*.test.tsx'], { query: '?raw', import: 'default', eager: true })).map(
    ([path, text]) => [path.replace(/^\.\.\//, ''), text as string],
  ),
)
const GUARDED_FILES = [
  'App.tsx',
  'tabs/MapNavTab.tsx',
  'poi/PoiPanels.tsx',
  'photos/PhotoList.tsx',
  'photos/PhotoPanel.tsx',
  'grasp/GraspPanel.tsx',
  'tabs/AgentChatTab.tsx',
  'tabs/RgbdCameraTab.tsx',
  'components/GpsChip.tsx',
  'overlays/OverlayBar.tsx',
]
const PANEL_BG_LITERAL = '#16181d'
const THEME_FILE = 'theme.ts'

/**
 * Allow-list of constants whose pixel values are not layout sizes: touch target, image request sizes, scroll thresholds.
 * A line mentioning one of these names is not checked.
 */
const ALLOWED_NAMES = [
  'TOUCH_TARGET_PX',
  'PANEL_IMAGE_PX',
  'THUMB_WIDTH_PX',
  'THUMB_MAX_PX',
  'STICK_THRESHOLD_PX',
  'RESULT_MAX_HEIGHT_PX',
  'THUMB_REQUEST_PX',
]

/** Hard-coded layout values from the brief (section 6), each as a pattern over one source line. */
const FORBIDDEN: { name: string; pattern: RegExp }[] = [
  { name: 'fixed width 260/300/360', pattern: /\bwidth:\s*(\{[^}]*\b)?(260|300|360)\b/ },
  { name: 'fixed maxHeight number', pattern: /\bmaxHeight:\s*(\{[^}]*\b)?\d+\b(?!\s*%)/ },
  { name: 'vh-only maxHeight', pattern: /\bmaxHeight:[^\n]*\d+vh/ },
  { name: 'calc(100% - 16px)', pattern: /calc\(100% - 16px\)/ },
  { name: 'fixed minHeight number >= 36', pattern: /\bminHeight:\s*(\{[^}]*\b)?([3-9]\d|\d{3,})\b/ },
  { name: 'fixed minWidth number', pattern: /\bminWidth:\s*(\{[^}]*\b)?([1-9]\d+)\b/ },
  { name: 'fixed maxWidth number', pattern: /\bmaxWidth:\s*(\{[^}]*\b)?([1-9]\d+)\b(?!\s*%)/ },
  { name: 'flex basis in px', pattern: /flex:\s*'[^']*\d+px/ },
  { name: 'MUI useMediaQuery(down(sm)) instead of useLayout', pattern: /useMediaQuery\(/ },
]

/**
 * @param file - path relative to src
 * @returns the source text
 */
function source(file: string): string {
  const text = SOURCES[file]
  if (text === undefined) throw new Error(`no source for ${file}`)
  return text
}

describe('hard-coded layout sizes', () => {
  for (const file of GUARDED_FILES) {
    it(`${file} has none of the brief's fixed layout values`, () => {
      const hits: string[] = []
      source(file)
        .split('\n')
        .forEach((line, i) => {
          if (/^\s*(\/\/|\*|\/\*)/.test(line) || ALLOWED_NAMES.some((n) => line.includes(n))) return
          for (const { name, pattern } of FORBIDDEN) if (pattern.test(line)) hits.push(`${file}:${i + 1} ${name}: ${line.trim()}`)
        })
      expect(hits).toEqual([])
    })
  }

  it('the PANEL_BG literal appears only in theme.ts', () => {
    const offenders = Object.keys(SOURCES).filter((f) => f !== THEME_FILE && SOURCES[f].includes(PANEL_BG_LITERAL))
    expect(offenders).toEqual([])
    expect(source(THEME_FILE)).toContain(PANEL_BG_LITERAL)
  })
})
