/**
 * Pure conversion of the 3D obstacle payloads (roles "cloud" and "scan_points", see bridge.POSE_FRAME_ROLES) into
 * map-frame positions and colours for the obstacles layer. Payload points stay in the sensor frame; "frame_pose" is
 * the pose of that frame in the map frame.
 */
import * as THREE from 'three'
import { SCENE_COLORS } from './palette'

/** Map-frame height bands (metres above the floor), matching the Nav2 ToF layers (design D2/D5). */
export const CLIFF_MAX_Z = -0.1
export const OBSTACLE_MIN_Z = 0.06
export const OBSTACLE_MAX_Z = 0.6

export const CLOUD_STRIDE = 3
export const SCAN_STRIDE = 2

export const TOF_CLIFF_COLOR = SCENE_COLORS.obstacleCliff
export const TOF_OBSTACLE_COLOR = SCENE_COLORS.obstacle
export const TOF_OTHER_COLOR = SCENE_COLORS.obstacleOther
export const LIDAR_COLOR = SCENE_COLORS.lidar

export interface FramePose {
  x: number
  y: number
  z: number
  qx: number
  qy: number
  qz: number
  qw: number
}

/** Serialized PointCloud2 (stride 3) or LaserScan (stride 2) with the pose of its frame in the map frame. */
export interface PointsPayload {
  stamp: number
  frame: string
  points: number[]
  frame_pose: FramePose
}

const POSE_KEYS: (keyof FramePose)[] = ['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']

/**
 * Validate a payload.
 *
 * @param data - raw topic value
 * @param stride - numbers per point (3 for the cloud, 2 for scan points)
 * @returns the payload, or null when it is missing, has no finite pose, or its points are not a finite multiple of stride
 */
export function parsePointsPayload(data: unknown, stride: number): PointsPayload | null {
  if (!data || typeof data !== 'object') return null
  const d = data as Partial<PointsPayload>
  if (!Array.isArray(d.points) || d.points.length % stride !== 0) return null
  if (!d.points.every((v) => Number.isFinite(v))) return null
  const pose = d.frame_pose
  if (!pose || typeof pose !== 'object' || !POSE_KEYS.every((k) => Number.isFinite(pose[k]))) return null
  return d as PointsPayload
}

/**
 * Transform sensor-frame points by the frame pose (quaternion rotation, then translation).
 *
 * @param pts - flat x, y, z of the sensor frame
 * @param pose - pose of the sensor frame in the map frame
 * @returns flat x, y, z in the map frame
 */
export function transformPoints(pts: ArrayLike<number>, pose: FramePose): Float32Array {
  const q = new THREE.Quaternion(pose.qx, pose.qy, pose.qz, pose.qw).normalize()
  const v = new THREE.Vector3()
  const out = new Float32Array(pts.length)
  for (let i = 0; i < pts.length; i += 3) {
    v.set(pts[i], pts[i + 1], pts[i + 2]).applyQuaternion(q)
    out[i] = v.x + pose.x
    out[i + 1] = v.y + pose.y
    out[i + 2] = v.z + pose.z
  }
  return out
}

/**
 * ToF cloud to map-frame positions.
 *
 * @param data - raw "cloud" topic value
 * @returns flat x, y, z in the map frame, or null for a malformed payload
 */
export function cloudToMapPoints(data: unknown): Float32Array | null {
  const p = parsePointsPayload(data, CLOUD_STRIDE)
  return p && transformPoints(p.points, p.frame_pose)
}

/**
 * Lidar scan points to map-frame positions; the scan plane is z = 0 in the scan frame.
 *
 * @param data - raw "scan_points" topic value
 * @returns flat x, y, z in the map frame, or null for a malformed payload
 */
export function scanToMapPoints(data: unknown): Float32Array | null {
  const p = parsePointsPayload(data, SCAN_STRIDE)
  if (!p) return null
  const xyz = new Array<number>((p.points.length / SCAN_STRIDE) * CLOUD_STRIDE).fill(0)
  for (let i = 0, j = 0; i < p.points.length; i += SCAN_STRIDE, j += CLOUD_STRIDE) {
    xyz[j] = p.points[i]
    xyz[j + 1] = p.points[i + 1]
  }
  return transformPoints(xyz, p.frame_pose)
}

/**
 * Per-point ToF colours by map-frame height: cliff (below the floor), obstacle band, everything else.
 *
 * @param xyz - flat x, y, z in the map frame
 * @returns flat r, g, b (0..1), one triple per point
 */
export function tofColors(xyz: ArrayLike<number>): Float32Array {
  const out = new Float32Array(xyz.length)
  const cliff = new THREE.Color(TOF_CLIFF_COLOR)
  const obstacle = new THREE.Color(TOF_OBSTACLE_COLOR)
  const other = new THREE.Color(TOF_OTHER_COLOR)
  for (let i = 0; i < xyz.length; i += 3) {
    const z = xyz[i + 2]
    const c = z <= CLIFF_MAX_Z ? cliff : z >= OBSTACLE_MIN_Z && z <= OBSTACLE_MAX_Z ? obstacle : other
    out[i] = c.r
    out[i + 1] = c.g
    out[i + 2] = c.b
  }
  return out
}
