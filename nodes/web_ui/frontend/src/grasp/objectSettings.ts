/** Object size settings of the Grasp panel, remembered per browser (localStorage; every access may throw). */
import type { TabStorage } from '../tabSelection'
import { DEFAULT_GRIP_PROFILE, GripProfileName, isGripProfile } from './grasp'

export const OBJECT_SETTINGS_KEY = 'grasp.object'
export const GRIP_PROFILE_KEY = 'grasp.gripProfile'

/** Text of the size fields (metres) and the Scoop gap below the object. */
export interface ObjectSettings {
  width: string
  depth: string
  height: string
  gapBelow: string
}

export const DEFAULT_OBJECT_SETTINGS: ObjectSettings = { width: '0.04', depth: '0.04', height: '0.04', gapBelow: '0' }

const KEYS = Object.keys(DEFAULT_OBJECT_SETTINGS) as (keyof ObjectSettings)[]

/**
 * Stored settings; any missing, corrupt or unreadable part falls back to the default.
 *
 * @param storage - localStorage or a stand-in, or undefined when unavailable
 * @returns the settings
 */
export function loadObjectSettings(storage: TabStorage | undefined): ObjectSettings {
  const settings = { ...DEFAULT_OBJECT_SETTINGS }
  try {
    const raw = storage?.getItem(OBJECT_SETTINGS_KEY)
    const parsed: unknown = raw ? JSON.parse(raw) : null
    if (typeof parsed === 'object' && parsed !== null) {
      for (const key of KEYS) {
        const value = (parsed as Record<string, unknown>)[key]
        if (typeof value === 'string') settings[key] = value
      }
    }
  } catch {
    // unavailable or corrupt: defaults
  }
  return settings
}

/**
 * Remember the settings; failures are ignored.
 *
 * @param storage - localStorage or a stand-in, or undefined when unavailable
 * @param settings - settings to store
 */
export function saveObjectSettings(storage: TabStorage | undefined, settings: ObjectSettings): void {
  try {
    storage?.setItem(OBJECT_SETTINGS_KEY, JSON.stringify(settings))
  } catch {
    // storage unavailable or full: not remembered
  }
}

/**
 * Stored grip strength; anything missing, unknown or unreadable gives the default (normal).
 *
 * @param storage - localStorage or a stand-in, or undefined when unavailable
 * @returns the grip profile name
 */
export function loadGripProfile(storage: TabStorage | undefined): GripProfileName {
  try {
    const raw = storage?.getItem(GRIP_PROFILE_KEY)
    return isGripProfile(raw) ? raw : DEFAULT_GRIP_PROFILE
  } catch {
    return DEFAULT_GRIP_PROFILE
  }
}

/**
 * Remember the grip strength; failures are ignored.
 *
 * @param storage - localStorage or a stand-in, or undefined when unavailable
 * @param profile - grip profile name
 */
export function saveGripProfile(storage: TabStorage | undefined, profile: GripProfileName): void {
  try {
    storage?.setItem(GRIP_PROFILE_KEY, profile)
  } catch {
    // storage unavailable or full: not remembered
  }
}
