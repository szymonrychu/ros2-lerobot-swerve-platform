/**
 * Pure helpers deciding which tab the dashboard opens on and in what order tabs are listed.
 *
 * The map (type ``map_nav``) is the primary tab: it is listed first and selected by default.
 * The viewer's last choice is remembered in localStorage, but only restored while it still exists.
 */

export const TAB_STORAGE_KEY = 'web_ui.activeTabId'
export const PRIMARY_TAB_TYPE = 'map_nav'
/** Tab types the dashboard renders; others (e.g. the merged-away nav_local, nav_gps, scene3d, robot_status, effector_graph) are dropped. */
export const SUPPORTED_TAB_TYPES: readonly string[] = ['map_nav', 'arm', 'camera', 'rgbd_camera', 'poi', 'sensor_graph', 'agent_chat']

/** Minimal tab shape the helpers need (a TabConfig satisfies it). */
export interface TabRef {
  id: string
  type: string
}

/** Storage subset used here, so tests can pass an in-memory stand-in for localStorage. */
export interface TabStorage {
  getItem(key: string): string | null
  setItem(key: string, value: string): void
}

/**
 * Drop tabs whose type the dashboard no longer renders (a stale config must not show dead tabs).
 *
 * @param tabs - tabs as configured (not mutated)
 * @returns a new array with only supported tab types, order kept
 */
export function supportedTabs<T extends TabRef>(tabs: readonly T[]): T[] {
  return tabs.filter((t) => SUPPORTED_TAB_TYPES.includes(t.type))
}

/**
 * Return the tabs with every map_nav tab first, keeping the relative order otherwise.
 *
 * @param tabs - tabs as configured (not mutated)
 * @returns a new array with the primary tabs leading
 */
export function orderTabs<T extends TabRef>(tabs: readonly T[]): T[] {
  return [...tabs.filter((t) => t.type === PRIMARY_TAB_TYPE), ...tabs.filter((t) => t.type !== PRIMARY_TAB_TYPE)]
}

/**
 * Pick the tab index to open on.
 *
 * @param tabs - tabs in display order
 * @param storedId - id the viewer selected last time, or null
 * @returns index of the stored tab if it still exists, else of the first map_nav tab, else 0
 */
export function initialTabIndex(tabs: readonly TabRef[], storedId: string | null): number {
  if (storedId !== null) {
    const stored = tabs.findIndex((t) => t.id === storedId)
    if (stored >= 0) return stored
  }
  const map = mapTabIndex(tabs)
  return map >= 0 ? map : 0
}

/**
 * Index of the map tab.
 *
 * @param tabs - tabs in display order
 * @returns index of the first map_nav tab, or -1 when none is configured
 */
export function mapTabIndex(tabs: readonly TabRef[]): number {
  return tabs.findIndex((t) => t.type === PRIMARY_TAB_TYPE)
}

/**
 * Read the remembered tab id; storage may be missing or throw (private mode, blocked site data).
 *
 * @param storage - localStorage or a stand-in, or undefined when unavailable
 * @returns the stored id, or null
 */
export function readStoredTabId(storage: TabStorage | undefined): string | null {
  try {
    return storage?.getItem(TAB_STORAGE_KEY) ?? null
  } catch {
    return null
  }
}

/**
 * Remember the selected tab id; failures are ignored (the map tab is the fallback anyway).
 *
 * @param storage - localStorage or a stand-in, or undefined when unavailable
 * @param id - id of the selected tab
 */
export function writeStoredTabId(storage: TabStorage | undefined, id: string): void {
  try {
    storage?.setItem(TAB_STORAGE_KEY, id)
  } catch {
    // Storage unavailable or full: selection simply is not remembered.
  }
}

/**
 * localStorage when the browser exposes it; accessing it can itself throw.
 *
 * @returns window.localStorage, or undefined when unavailable
 */
export function browserStorage(): TabStorage | undefined {
  try {
    return typeof window === 'undefined' ? undefined : window.localStorage
  } catch {
    return undefined
  }
}
