/** Object size settings of the Grasp panel, remembered per browser (localStorage; every access may throw). */
import type { TabStorage } from '../tabSelection'

export const OBJECT_SETTINGS_KEY = 'grasp.object'

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
