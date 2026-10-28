/**
 * The one map palette: every colour of the map, the 3D scene and the POI and photo markers.
 *
 * Only three things carry a hue: the keepout zone (TOKENS.error), the slow zone (TOKENS.warning) and the navigation path.
 * Everything else is greyscale, so those stand out. POI kinds are told apart by shape, POI status by a lightness step.
 */
import type { LayerKey } from './layers'
import { TOKENS } from '../theme'

/** Colour of the navigation paths (global and local); the only non-zone hue on the map. */
export const NAV_PATH_COLOR = '#2ecc40'

export const ZONE_COLORS = { keepout: TOKENS.error, slow: TOKENS.warning } as const

/** POI status outline on markers: a neutral lightness step (open brightest, cancelled dimmest). */
export const STATUS_COLORS = { open: '#f2f2f2', done: '#a6a6a6', cancelled: '#666666' } as const
/** Marker outline of the creator: agent grey (and dashed), user white. */
export const CREATOR_OUTLINE = { agent: '#999999', user: '#ffffff' } as const
/** Outline of the selected POI. */
export const SELECTED_OUTLINE: string = TOKENS.accent

/** Fill of remembered objects. */
export const OBJECT_COLOR = '#b8b8b8'
/** Fill of photo markers and viewing wedges, one colour for every camera. */
export const PHOTO_COLOR = '#8c8c8c'
/** Marker fill by POI kind. area-none is the neutral fill of an area without a zone. */
export const KIND_COLORS = { point: '#dcdcdc', object: OBJECT_COLOR, photo: PHOTO_COLOR, marker: '#f4f4f4', 'area-none': '#707070' } as const

/** Colours of the 3D map scene. */
export const SCENE_COLORS = {
  robot: '#cfcfcf',
  robotFront: '#ffffff',
  globalPath: NAV_PATH_COLOR,
  localPath: NAV_PATH_COLOR,
  goal: '#ffffff',
  rtkBase: '#bfbfbf',
  draft: '#9a9a9a',
  costmap: '#8a8a8a',
  free: '#fefefe',
  occupied: '#000000',
  unknown: '#cdcdcd',
  obstacleCliff: TOKENS.error, // 3D obstacles layer: D435 depth points below the floor (reuses the keepout hue)
  obstacle: TOKENS.warning, // D435 depth points in the 0.10-0.6 m obstacle band (reuses the slow-zone hue)
  obstacleOther: '#8a8a8a',
  lidar: '#f2f2f2',
  map3dPoints: '#e6e6e6', // measured 3D map points without a colour
  lidarWall: '#6e6e6e', // translucent lidar-derived wall boxes, never used for measured points
  gridCentre: '#2a2a2a',
  gridLine: '#1a1a1a',
} as const

/** Colours of the Ground layer, also the legend of the layers panel. */
export const GROUND_COLORS = {
  floor: '#2e9e4f',
  obstacle: '#d32f2f',
  drop: '#1e66f5',
  mismatch: '#f57c00',
  unlabelled: '#9e9e9e',
  edgeObstacle: '#d32f2f',
  edgeWarning: '#fdd835',
  plane: '#2e9e4f',
  warningFill: '#fdd835',
  noReturnFill: '#8e24aa',
} as const

/** Layer toolbar swatches (kept here so the toolbar does not pull in the 3D chunk). */
export const LAYER_SWATCH: Partial<Record<LayerKey, string>> = {
  localCostmap: SCENE_COLORS.costmap,
  globalPlan: SCENE_COLORS.globalPath,
  localPlan: SCENE_COLORS.localPath,
  goal: SCENE_COLORS.goal,
  rtkBase: SCENE_COLORS.rtkBase,
  footprint: SCENE_COLORS.robot,
  pois: KIND_COLORS.point,
  photos: PHOTO_COLOR,
  map3dCloud: SCENE_COLORS.map3dPoints,
  lidarWalls: SCENE_COLORS.lidarWall,
}

export const MAP_LEGEND: [string, string][] = [
  ['Free', SCENE_COLORS.free],
  ['Occupied', SCENE_COLORS.occupied],
  ['Unknown', SCENE_COLORS.unknown],
  ['Robot front', SCENE_COLORS.robotFront],
  ['Goal being set', SCENE_COLORS.draft],
]

/** Colours of the arm views (hex numbers for three.js): greyscale, states told apart by lightness; axes by lightness and their label. */
export const ARM_COLORS = {
  handle: { idle: 0xb0b0b0, hover: 0xd4d4d4, drag: 0xffffff },
  endEffector: { idle: 0x9a9a9a, hover: 0xc0c0c0, drag: 0xeeeeee },
  axis: { x: 0xe0e0e0, y: 0xa8a8a8, z: 0x707070 },
  ghost: 0xcccccc,
  mesh: 0x888888,
} as const

/** Sensor graph chrome: light grey axis text, faint white grid and ticks. */
export const AXIS_COLOR = '#9a9a9a'
export const GRID_COLOR = 'rgba(255, 255, 255, 0.08)'
/** Sensor graph series colours, greyscale steps used in order when a field sets no colour of its own. */
export const SERIES_COLORS = ['#f2f2f2', '#b3b3b3', '#808080', '#d9d9d9', '#999999', '#666666'] as const
