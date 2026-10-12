import { describe, expect, it } from 'vitest'
import { SOURCES, designViolations } from './designRules'

const PATH = 'tabs/Example.tsx'

/** Rule ids named in a violation list, e.g. ["colour", "weight"]. */
const rulesOf = (source: string, path = PATH): string[] => designViolations(path, source).map((m) => m.split(':')[0])

describe('designViolations rule fixtures', () => {
  it('(a) colour literals', () => {
    for (const bad of [
      "const c = '#fff'",
      "const c = '#1a2b3c'",
      "const c = 'rgb(1, 2, 3)'",
      "const c = 'rgba(1,2,3,.5)'",
      "const c = 'hsl(10, 20%, 30%)'",
      'const c = 0xff8800',
    ]) {
      expect(rulesOf(bad), bad).toEqual(['colour'])
    }
    expect(rulesOf("const c = theme.palette.text.primary; const n = 'value0'")).toEqual([])
    expect(rulesOf("const c = '#fff'", 'theme.ts')).toEqual([])
    expect(rulesOf('const c = 0xff8800', 'map3d/palette.ts')).toEqual([])
  })

  it('(b) shadows, blur and gradients', () => {
    for (const bad of ['sx={{ boxShadow: 1 }}', "sx={{ backdropFilter: 'blur(4px)' }}", "sx={{ textShadow: 'none' }}", "sx={{ background: 'linear-gradient(a, b)' }}"]) {
      expect(rulesOf(bad), bad).toEqual(['effect'])
    }
    expect(rulesOf("sx={{ bgcolor: 'background.paper', border: 1 }}")).toEqual([])
  })

  it('(c) color="secondary"', () => {
    expect(rulesOf('<Chip color="secondary" />')).toEqual(['secondary'])
    expect(rulesOf("<Chip color={'secondary'} />")).toEqual(['secondary'])
    expect(rulesOf('<Chip color="warning" />')).toEqual([])
    expect(rulesOf("<Typography color='text.secondary' />")).toEqual([])
  })

  it('(d) filled Alert, also across lines', () => {
    expect(rulesOf('<Alert severity="error" variant="filled">x</Alert>')).toEqual(['alert'])
    expect(rulesOf('<Alert\n  severity="error"\n  onClose={() => go()}\n  variant="filled"\n>x</Alert>')).toEqual(['alert'])
    expect(rulesOf('<Alert severity="error" variant="outlined">x</Alert>')).toEqual([])
    expect(rulesOf('<Chip variant="filled" />')).toEqual([])
  })

  it('(e) font weight 400 and 500 only', () => {
    for (const bad of ['fontWeight: 600', 'fontWeight: 700', 'fontWeight: 800', "fontWeight: 'bold'", "sx={{ fontWeight: '700' }}", 'fontWeight={600}', 'fontWeight: 300']) {
      expect(rulesOf(bad), bad).toEqual(['weight'])
    }
    for (const ok of ['fontWeight: 400', 'fontWeight: 500', "fontWeight: '500'", 'fontWeight: medium', 'fontWeight,']) {
      expect(rulesOf(ok), ok).toEqual([])
    }
  })

  it('(f) off-grid spacing in spacing keys', () => {
    for (const bad of ['sx={{ p: 1.5 }}', 'sx={{ gap: 0.75 }}', 'sx={{ mt: 2.5 }}', 'sx={{ px: 1.25 }}', 'sx={{ p: { xs: 0.75, sm: 2 } }}', 'sx={{ marginBottom: 1.5 }}', '<Stack spacing={1.5}>']) {
      expect(rulesOf(bad), bad).toEqual(['spacing'])
    }
    for (const ok of ['sx={{ p: 1, gap: 2, mt: 0.5 }}', 'sx={{ p: { xs: 1, sm: 2 } }}', 'const x = { opacity: 1.5 }', 'sx={{ lineHeight: 1.5 }}']) {
      expect(rulesOf(ok), ok).toEqual([])
    }
  })

  it('(g) decorative unicode in strings and JSX text', () => {
    for (const bad of [
      "const s = 'a — b'",
      "const s = 'wait…'",
      "const s = 'go → there'",
      "const s = '⇒'",
      "const s = '● live'",
      "const s = '○ idle'",
      '<b>\u{1F680} go</b>',
      "const s = '✓ ok'",
      "const s = '“quoted”'",
    ]) {
      expect(rulesOf(bad), bad).toEqual(['unicode'])
    }
    for (const ok of ["const s = '45°'", "const s = 'Plain text - with hyphen...'", '<b>Zolta lodz</b>']) {
      expect(rulesOf(ok), ok).toEqual([])
    }
  })

  it('(h) title= on Button and IconButton', () => {
    expect(rulesOf('<Button title="Go" onClick={go}>x</Button>')).toEqual(['title'])
    expect(rulesOf('<IconButton title="Go"><X /></IconButton>')).toEqual(['title'])
    expect(rulesOf('<IconButton\n  onClick={() => go()}\n  title="Go"\n>\n  <X />\n</IconButton>')).toEqual(['title'])
    expect(rulesOf('<IconButton aria-label="Go"><X /></IconButton>')).toEqual([])
    expect(rulesOf('<Tooltip title="Go"><IconButton aria-label="Go" /></Tooltip>')).toEqual([])
  })

  it('names the line of the violation', () => {
    const [msg] = designViolations(PATH, "a\nb\nconst c = '#fff'")
    expect(msg).toContain('line 3')
    expect(msg).toContain(PATH)
  })
})

describe('designViolations on shipped sources', () => {
  it('SOURCES excludes tests and includes the entry files', () => {
    expect(SOURCES['main.tsx']).toBeDefined()
    expect(SOURCES['theme.ts']).toBeDefined()
    expect(Object.keys(SOURCES).some((f) => f.endsWith('.test.ts') || f.endsWith('.test.tsx'))).toBe(false)
  })

  it.each(['theme.ts', 'main.tsx'])('%s has no violations', (file) => {
    expect(designViolations(file, SOURCES[file])).toEqual([])
  })
})
