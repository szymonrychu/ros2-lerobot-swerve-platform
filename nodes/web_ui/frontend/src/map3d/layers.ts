/** Layer visibility state of the 3D map tab, persisted per browser in localStorage. */
import type { TabStorage } from '../tabSelection'
import type { TabConfig } from '../types'

export const LAYER_STORAGE_KEY = 'web_ui.map3d.layers'

export const LAYER_KEYS = [
  'slamMap',
  'localCostmap',
  'gpsMap',
  'rtkBase',
  'globalPlan',
  'localPlan',
  'goal',
  'footprint',
  'robotBase',
  'robotWheels',
  'robotArm',
  'head',
  'pois',
  'photos',
  'obstacles3d',
  'map3dCloud',
  'headRgbd',
  'ground',
  'lidarWalls',
] as const

export type LayerKey = (typeof LAYER_KEYS)[number]
export type LayerState = Record<LayerKey, boolean>

/** Layers that start hidden: the GPS map (it needs an anchor and fetches tiles) and the on-demand 3D obstacles, head RGB-D cloud and ground estimate. */
const DEFAULT_OFF: readonly LayerKey[] = ['gpsMap', 'obstacles3d', 'headRgbd', 'ground']

/** Everything visible except the DEFAULT_OFF layers. */
export const DEFAULT_LAYERS: Readonly<LayerState> = Object.freeze(
  Object.fromEntries(LAYER_KEYS.map((k) => [k, !DEFAULT_OFF.includes(k)])) as LayerState,
)

/** Human labels, in panel order. */
export const LAYER_LABELS: Record<LayerKey, string> = {
  slamMap: 'SLAM map',
  localCostmap: 'Local costmap',
  gpsMap: 'GPS map',
  rtkBase: 'RTK base',
  globalPlan: 'Global plan',
  localPlan: 'Local plan',
  goal: 'Goal',
  footprint: 'Footprint',
  robotBase: 'Robot body',
  robotWheels: 'Wheels',
  robotArm: 'Arm',
  head: 'Head',
  pois: 'POIs',
  photos: 'Photos',
  obstacles3d: '3D obstacles (D435 + lidar)',
  map3dCloud: '3D map (measured)',
  headRgbd: 'Head RGB-D',
  ground: 'Ground',
  lidarWalls: 'Lidar walls (2D only)',
}

/** Tab config fields naming the topics a layer needs; they are subscribed only while the layer is on. */
export const LAYER_TOPIC_FIELDS: Partial<Record<LayerKey, readonly (keyof TabConfig)[]>> = {
  obstacles3d: ['obstacle_cloud_topic', 'scan_points_topic'],
  map3dCloud: ['map3d_cloud_topic'],
  headRgbd: ['rgbd_cloud_topic'],
  ground: ['ground_cloud_topic', 'ground_state_topic'],
}

/**
 * Parse stored layer state.
 *
 * @param raw - stored JSON string, or null
 * @returns a fresh LayerState: defaults overridden by every stored boolean of a known key
 */
export function parseLayerState(raw: string | null): LayerState {
  const state: LayerState = { ...DEFAULT_LAYERS }
  if (raw === null) return state
  let parsed: unknown
  try {
    parsed = JSON.parse(raw)
  } catch {
    return state
  }
  if (!parsed || typeof parsed !== 'object' || Array.isArray(parsed)) return state
  const obj = parsed as Record<string, unknown>
  for (const k of LAYER_KEYS) {
    if (typeof obj[k] === 'boolean') state[k] = obj[k] as boolean
  }
  return state
}

/**
 * Read layer state; storage may be missing or throw.
 *
 * @param storage - localStorage or a stand-in, or undefined
 * @returns stored state, or the defaults
 */
export function readLayerState(storage: TabStorage | undefined): LayerState {
  try {
    return parseLayerState(storage?.getItem(LAYER_STORAGE_KEY) ?? null)
  } catch {
    return parseLayerState(null)
  }
}

/**
 * Persist layer state; failures are ignored.
 *
 * @param storage - localStorage or a stand-in, or undefined
 * @param state - state to store
 */
export function writeLayerState(storage: TabStorage | undefined, state: LayerState): void {
  try {
    storage?.setItem(LAYER_STORAGE_KEY, JSON.stringify(state))
  } catch {
    // Storage unavailable or full: layers simply reset next visit.
  }
}
