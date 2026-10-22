/**
 * Ground layer data: the labelled ground cloud and the ground state JSON of ground_estimator_cpp, turned into
 * what the 3D map draws (label colours, plane patch, edge segments, warning polygons). Pure, no three.js.
 */
import { decodeMap3dCloud } from './cloudDecode'
import { GROUND_COLORS } from './palette'
import { isFramePose, transformPoints, type FramePose } from './obstaclePoints'
import type { Map3dCloudPayload } from '../types'

/** Label values of the ground cloud (ground_estimator_cpp colour.hpp Label). */
export const GROUND_LABEL = { floor: 0, aboveFloor: 1, belowFloor: 2, floorColourMismatch: 3, unlabelled: 4 } as const

export { GROUND_COLORS }

/** Legend rows of the five label colours, in label order. */
export const GROUND_LABEL_LEGEND: readonly (readonly [string, string])[] = [
  ['Floor', GROUND_COLORS.floor],
  ['Obstacle (above floor)', GROUND_COLORS.obstacle],
  ['Drop (below floor)', GROUND_COLORS.drop],
  ['Floor colour mismatch', GROUND_COLORS.mismatch],
  ['Unlabelled', GROUND_COLORS.unlabelled],
]

/** Legend rows of the edge lines and the warning area fill. */
export const GROUND_EDGE_LEGEND: readonly (readonly [string, string])[] = [
  ['Obstacle edge', GROUND_COLORS.edgeObstacle],
  ['Warning edge', GROUND_COLORS.edgeWarning],
  ['Warning area (slow down)', GROUND_COLORS.warningFill],
  ['No ToF return (caution)', GROUND_COLORS.noReturnFill],
]

const LABEL_COLOURS: readonly string[] = [
  GROUND_COLORS.floor,
  GROUND_COLORS.obstacle,
  GROUND_COLORS.drop,
  GROUND_COLORS.mismatch,
  GROUND_COLORS.unlabelled,
]

/** Side of the plane patch and length of an edge line, metres. */
export const PATCH_SIZE_M = 2
export const EDGE_LENGTH_M = 2
const MIN_NORMAL_Z = 1e-6
const HEX_RADIX = 16
const BYTE_MAX = 255

export type Vec2Tuple = [number, number]
export type Vec3Tuple = [number, number, number]

/** The plane of the state, in base_link. */
export interface GroundPlane {
  normal: Vec3Tuple
  height_m: number
  inliers: number
  inlier_ratio: number
  source: string
  age_s: number
}

/** One region of the state (mcp SurfaceRegion shape); class is "obstacle" or "warning". */
export interface GroundRegion {
  name: string
  kind: string
  class: string
  height_m: number
  frame: string
  edge: { point: Vec2Tuple; direction: Vec2Tuple; side: string }
  distance_m: number
  points: number
}

/** One warning area of the state. */
export interface GroundWarningArea {
  kind: string
  polygon: Vec2Tuple[]
  height_m: number | null
  slope_deg: number | null
  source: string
}

/** The ground state JSON; keys equal STATE_KEYS of ground_estimator_cpp (state.hpp), plus "stale" set by the backend. */
export interface GroundState {
  stamp_ns: number
  frame: string
  prior: unknown
  imu_ok: boolean
  plane: GroundPlane | null
  reason: string | null
  slope_deg: number | null
  slope_roll_deg: number | null
  slope_pitch_deg: number | null
  slope_level: string | null
  base_tilt_deg: number | null
  plane_vs_base_deg: number | null
  traversability: unknown
  obstacle_points: number
  drop_points: number
  regions: GroundRegion[]
  warnings: GroundWarningArea[]
  colour: unknown
  stale?: boolean
}

/** ground_cloud payload: the map3d cloud payload plus uint8 labels, the header frame and the pose of that frame in the map. */
export interface GroundCloudPayload extends Map3dCloudPayload {
  labels_b64?: string
  frame?: string
  frame_pose?: FramePose
}

/** A decoded ground cloud in the map frame (ROS axes). */
export interface GroundCloud {
  positions: Float32Array
  /** The point colours of the cloud, 0..1, when it had them. */
  colors?: Float32Array
  labels: Uint8Array
  /** One label colour per point, 0..1. */
  labelColors: Float32Array
  count: number
}

/** An edge segment of a region, base_link xy. */
export interface EdgeSegment {
  name: string
  kind: string
  cls: string
  colour: string
  a: Vec2Tuple
  b: Vec2Tuple
}

/** A warning area polygon, base_link xy. */
export interface WarningPolygon {
  kind: string
  source: string
  polygon: Vec2Tuple[]
}

/** Height and slope of one warning area, as text ("n/a" for null). */
export interface WarningAreaReadout {
  source: string
  kind: string
  height: string
  slope: string
}

/** Slope, slope level and plane height of a state, as text. */
export interface GroundReadout {
  slope: string
  level: string
  height: string
}

/**
 * @param label - uint8 label of a point
 * @returns the palette colour (hex); anything unknown is the unlabelled grey
 */
export function labelColour(label: number): string {
  return LABEL_COLOURS[label] ?? GROUND_COLORS.unlabelled
}

const RGB_CACHE = new Map<string, [number, number, number]>()

function rgbOf(hex: string): [number, number, number] {
  let rgb = RGB_CACHE.get(hex)
  if (!rgb) {
    rgb = [1, 3, 5].map((i) => parseInt(hex.slice(i, i + 2), HEX_RADIX) / BYTE_MAX) as [number, number, number]
    RGB_CACHE.set(hex, rgb)
  }
  return rgb
}

function bytesOf(b64: string): Uint8Array | null {
  try {
    const bin = atob(b64)
    return Uint8Array.from(bin, (c) => c.charCodeAt(0))
  } catch {
    return null
  }
}

/**
 * Decode a ground_cloud payload into map-frame points with per-point label colours.
 *
 * @param payload - raw topic value (points via decodeMap3dCloud, labels_b64, frame_pose)
 * @returns the cloud, or null for a malformed payload, a missing or non-finite frame_pose or labels that do not match count; without labels_b64 every point is unlabelled
 */
export function decodeGroundCloud(payload: unknown): GroundCloud | null {
  if (!payload || typeof payload !== 'object') return null
  const p = payload as Partial<GroundCloudPayload>
  if (!isFramePose(p.frame_pose)) return null
  const cloud = decodeMap3dCloud(payload, undefined, { anyFrame: true, rosAxes: true })
  if (!cloud) return null
  let labels: Uint8Array
  if (typeof p.labels_b64 === 'string') {
    const decoded = bytesOf(p.labels_b64)
    if (!decoded || decoded.length !== cloud.count) return null
    labels = decoded
  } else {
    labels = new Uint8Array(cloud.count).fill(GROUND_LABEL.unlabelled)
  }
  const labelColors = new Float32Array(cloud.count * 3)
  labels.forEach((l, i) => labelColors.set(rgbOf(labelColour(l)), i * 3))
  return { positions: transformPoints(cloud.positions, p.frame_pose), colors: cloud.colors, labels, labelColors, count: cloud.count }
}

/**
 * @param state - ground state or nothing yet
 * @returns false for an absent, stale or plane-null state: nothing may be drawn from it
 */
export function groundVisible(state: GroundState | null | undefined): state is GroundState {
  return Boolean(state) && state!.stale !== true && Boolean(state!.plane)
}

/**
 * Plane height at a base_link xy.
 *
 * @param plane - the state plane
 * @param x - base_link x
 * @param y - base_link y
 * @returns z of the plane there, or null for a (near) vertical normal
 */
export function planeZ(plane: GroundPlane, x: number, y: number): number | null {
  const [nx, ny, nz] = plane.normal
  if (!Number.isFinite(nz) || Math.abs(nz) < MIN_NORMAL_Z) return null
  return plane.height_m - (nx * x + ny * y) / nz
}

/**
 * The 2 m x 2 m square around the base_link origin on the state plane.
 *
 * @param state - ground state
 * @returns four corners (x, y, z) in base_link, or null without a plane (or with a vertical normal)
 */
export function planePatch(state: GroundState): Vec3Tuple[] | null {
  const plane = state.plane
  if (!plane) return null
  const half = PATCH_SIZE_M / 2
  const corners: Vec3Tuple[] = []
  for (const [x, y] of [[-half, -half], [half, -half], [half, half], [-half, half]]) {
    const z = planeZ(plane, x, y)
    if (z === null) return null
    corners.push([x, y, z])
  }
  return corners
}

/**
 * @param state - ground state
 * @returns the warning area polygons (base_link xy, at least three vertices) with kind and source; none for an invisible state
 */
export function warningPolygons(state: GroundState | null | undefined): WarningPolygon[] {
  if (!groundVisible(state)) return []
  return (state.warnings ?? [])
    .filter((w) => Array.isArray(w.polygon) && w.polygon.length >= 3)
    .map((w) => ({ kind: w.kind, source: w.source, polygon: w.polygon }))
}

/**
 * @param state - ground state
 * @returns per region a segment EDGE_LENGTH_M long, centred on the edge point along the direction; red for class obstacle, yellow otherwise; none for an invisible state
 */
export function regionEdges(state: GroundState | null | undefined): EdgeSegment[] {
  if (!groundVisible(state)) return []
  const half = EDGE_LENGTH_M / 2
  return (state.regions ?? []).map((r) => {
    const [px, py] = r.edge.point
    const [dx, dy] = r.edge.direction
    return {
      name: r.name,
      kind: r.kind,
      cls: r.class,
      colour: r.class === 'obstacle' ? GROUND_COLORS.edgeObstacle : GROUND_COLORS.edgeWarning,
      a: [px - dx * half, py - dy * half],
      b: [px + dx * half, py + dy * half],
    }
  })
}

/**
 * @param state - ground state
 * @returns slope, slope level and plane height as text, or null for an invisible state
 */
export function groundReadout(state: GroundState | null | undefined): GroundReadout | null {
  if (!groundVisible(state)) return null
  return {
    slope: state.slope_deg === null ? 'n/a' : `${state.slope_deg.toFixed(1)} deg`,
    level: state.slope_level ?? 'n/a',
    height: `${state.plane!.height_m.toFixed(3)} m`,
  }
}

/**
 * @param kind - warning area kind
 * @returns the fill colour (hex): purple for no_return, yellow for every other kind
 */
export function warningAreaColour(kind: string): string {
  return kind === 'no_return' ? GROUND_COLORS.noReturnFill : GROUND_COLORS.warningFill
}

/**
 * @param state - ground state
 * @returns per warning area the height and slope as text, "n/a" for null; none for an invisible state
 */
export function warningAreaReadouts(state: GroundState | null | undefined): WarningAreaReadout[] {
  if (!groundVisible(state)) return []
  return (state.warnings ?? []).map((w) => ({
    source: w.source,
    kind: w.kind,
    height: w.height_m === null || w.height_m === undefined ? 'n/a' : `${w.height_m.toFixed(3)} m`,
    slope: w.slope_deg === null || w.slope_deg === undefined ? 'n/a' : `${w.slope_deg.toFixed(1)} deg`,
  }))
}
