import { describe, expect, it } from 'vitest'
import PY from '../../../../../shared/ros2_common/poi_categories.py?raw'
import { CATEGORIES, PROBLEM_TYPES, parsePoiTags, stripPoiTags } from './categories'

/** Read a string tuple constant from the Python grammar file. */
function pyTuple(name: string): string[] {
  const m = new RegExp(`^${name} = \\(([^)]*)\\)`, 'm').exec(PY)
  if (!m) throw new Error(`${name} not found`)
  return [...m[1].matchAll(/"([^"]+)"/g)].map((x) => x[1])
}

describe('constants', () => {
  it('equal the Python tuples', () => {
    expect([...CATEGORIES]).toEqual(pyTuple('CATEGORIES'))
    expect([...PROBLEM_TYPES]).toEqual(pyTuple('PROBLEM_TYPES'))
  })
})

describe('parsePoiTags', () => {
  it.each(CATEGORIES.filter((c) => c !== 'problem'))('parses #%s', (c) => {
    expect(parsePoiTags(`#${c} text`)).toEqual({ category: c, problemType: null, survey: false, surveySpot: false })
  })
  it.each(PROBLEM_TYPES)('parses #problem/%s', (t) => {
    expect(parsePoiTags(`#problem/${t}`)).toEqual({ category: 'problem', problemType: t, survey: false, surveySpot: false })
  })
  it('reads survey tags and only the first line', () => {
    expect(parsePoiTags('#target #survey #survey-spot')).toMatchObject({ category: 'target', survey: true, surveySpot: true })
    expect(parsePoiTags('hello\n#target')).toEqual({ category: null, problemType: null, survey: false, surveySpot: false })
  })
  it('ignores bare #problem, unknown types and takes the first category', () => {
    expect(parsePoiTags('#problem').category).toBeNull()
    expect(parsePoiTags('#problem/nope').category).toBeNull()
    expect(parsePoiTags('#slow #target').category).toBe('slow')
  })
})

describe('stripPoiTags', () => {
  it('removes the tag line, keeps free text', () => {
    expect(stripPoiTags('#target\nbody')).toBe('body')
    expect(stripPoiTags('#problem/window glass wall\nmore')).toBe('glass wall\nmore')
    expect(stripPoiTags('#slow #survey')).toBe('')
    expect(stripPoiTags('plain\n#target')).toBe('plain\n#target')
  })
})
