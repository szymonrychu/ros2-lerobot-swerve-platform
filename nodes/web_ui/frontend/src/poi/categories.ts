/** POI category tags carried on the first line of a POI note. Mirror of shared/ros2_common/poi_categories.py. */

export const CATEGORIES = ['target', 'slow', 'keepout', 'problem', 'detail'] as const
export const PROBLEM_TYPES = [
  'window',
  'mirror',
  'glass',
  'tight',
  'overhang',
  'low_obstacle',
  'drop',
  'cable',
  'soft_floor',
  'other',
] as const

export type PoiCategory = (typeof CATEGORIES)[number]
export type ProblemType = (typeof PROBLEM_TYPES)[number]

export const SURVEY_TAG = '#survey'
export const SURVEY_SPOT_TAG = '#survey-spot'
const PROBLEM_PREFIX = '#problem/'

/** Chip label of each category. */
export const CATEGORY_LABELS: Record<PoiCategory, string> = {
  target: 'Target',
  slow: 'Slow zone',
  keepout: 'Keep-out',
  problem: 'Problem',
  detail: 'Detail',
}

export interface PoiTags {
  category: PoiCategory | null
  problemType: ProblemType | null
  survey: boolean
  surveySpot: boolean
}

function isCategory(v: string): v is PoiCategory {
  return (CATEGORIES as readonly string[]).includes(v)
}

function isProblemType(v: string): v is ProblemType {
  return (PROBLEM_TYPES as readonly string[]).includes(v)
}

function categoryTag(token: string): boolean {
  if (token.startsWith(PROBLEM_PREFIX)) return isProblemType(token.slice(PROBLEM_PREFIX.length))
  return token.startsWith('#') && isCategory(token.slice(1)) && token !== '#problem'
}

function knownTag(token: string): boolean {
  return categoryTag(token) || token === SURVEY_TAG || token === SURVEY_SPOT_TAG
}

function splitFirstLine(note: string): [string[], string] {
  const i = note.indexOf('\n')
  const first = i < 0 ? note : note.slice(0, i)
  return [first.split(/\s+/).filter(Boolean), i < 0 ? '' : note.slice(i + 1)]
}

/**
 * Read the tags on the first line of a note.
 *
 * @param note - the POI note
 * @returns parsed tags; unknown words are ignored, the first category tag wins
 */
export function parsePoiTags(note: string): PoiTags {
  const [tokens] = splitFirstLine(note)
  const tags: PoiTags = {
    category: null,
    problemType: null,
    survey: tokens.includes(SURVEY_TAG),
    surveySpot: tokens.includes(SURVEY_SPOT_TAG),
  }
  const token = tokens.find(categoryTag)
  if (token === undefined) return tags
  if (token.startsWith(PROBLEM_PREFIX)) {
    tags.category = 'problem'
    tags.problemType = token.slice(PROBLEM_PREFIX.length) as ProblemType
  } else {
    tags.category = token.slice(1) as PoiCategory
  }
  return tags
}

/**
 * The free text of a note.
 *
 * @param note - the POI note
 * @returns the note without known tags; unknown first-line words and later lines stay
 */
export function stripPoiTags(note: string): string {
  const [tokens, rest] = splitFirstLine(note)
  if (!tokens.some(knownTag)) return note
  const line = tokens.filter((t) => !knownTag(t)).join(' ')
  return line && note.includes('\n') ? `${line}\n${rest}` : line || rest
}
