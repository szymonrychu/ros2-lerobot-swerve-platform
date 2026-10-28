import { describe, expect, it, vi } from 'vitest'
import {
  CLIFF_MAX_Z,
  LIDAR_COLOR,
  OBSTACLE_MAX_Z,
  OBSTACLE_MIN_Z,
  cloudToMapPoints,
  rgbdCloudToMap,
  parsePointsPayload,
  scanToMapPoints,
  depthColors,
  type PointsPayload,
} from './obstaclePoints'

vi.mock('../logging', () => ({ default: { warn: vi.fn(), info: vi.fn(), debug: vi.fn() } }))

const IDENTITY = { x: 0, y: 0, z: 0, qx: 0, qy: 0, qz: 0, qw: 1 }
const YAW90 = { x: 1, y: 2, z: 0.5, qx: 0, qy: 0, qz: Math.SQRT1_2, qw: Math.SQRT1_2 }

function payload(points: number[], pose = IDENTITY): PointsPayload {
  return { stamp: 1, frame: 'd435_depth_optical_frame', points, frame_pose: pose }
}

describe('parsePointsPayload', () => {
  it('accepts a well-formed payload', () => {
    expect(parsePointsPayload(payload([1, 2, 3]), 3)).not.toBeNull()
    expect(parsePointsPayload(payload([1, 2]), 2)).not.toBeNull()
    expect(parsePointsPayload(payload([]), 3)).not.toBeNull()
  })
  it('rejects odd or mismatched lengths and malformed input', () => {
    expect(parsePointsPayload(payload([1, 2, 3, 4]), 3)).toBeNull()
    expect(parsePointsPayload(payload([1, 2, 3]), 2)).toBeNull()
    expect(parsePointsPayload(payload([1, 2, 3, 4, 5]), 2)).toBeNull()
    expect(parsePointsPayload(null, 3)).toBeNull()
    expect(parsePointsPayload(undefined, 3)).toBeNull()
    expect(parsePointsPayload({ points: [1, 2, 3] }, 3)).toBeNull()
    expect(parsePointsPayload({ ...payload([1, 2, 3]), frame_pose: undefined }, 3)).toBeNull()
    expect(parsePointsPayload({ ...payload([1, 2, 3]), points: 'x' }, 3)).toBeNull()
    expect(parsePointsPayload(payload([1, NaN, 3]), 3)).toBeNull()
  })
})

describe('cloudToMapPoints', () => {
  it('leaves points unchanged under the identity pose', () => {
    expect(Array.from(cloudToMapPoints(payload([1, 2, 3, -1, 0, 0.5]))!)).toEqual([1, 2, 3, -1, 0, 0.5])
  })
  it('applies a 90 deg yaw plus translation', () => {
    const out = cloudToMapPoints(payload([1, 0, 0, 0, 1, 2], YAW90))!
    const exp = [1, 3, 0.5, 0, 2, 2.5]
    exp.forEach((v, i) => expect(out[i]).toBeCloseTo(v, 6))
  })
  it('returns null for malformed payloads', () => {
    expect(cloudToMapPoints(payload([1, 2, 3, 4]))).toBeNull()
    expect(cloudToMapPoints(null)).toBeNull()
  })
})

describe('scanToMapPoints', () => {
  it('puts scan points at z = 0 in the scan frame, then transforms', () => {
    const id = scanToMapPoints(payload([1, 2, 3, 4]))!
    expect(Array.from(id)).toEqual([1, 2, 0, 3, 4, 0])
    const out = scanToMapPoints(payload([1, 0], YAW90))!
    ;[1, 3, 0.5].forEach((v, i) => expect(out[i]).toBeCloseTo(v, 6))
  })
  it('rejects a 3-stride payload', () => {
    expect(scanToMapPoints(payload([1, 2, 3]))).toBeNull()
  })
})

describe('depthColors', () => {
  it('uses the Nav2 depth_voxel_layer bands (nav2_params.yaml depth_mark, depth_cliff_mark)', () => {
    expect(CLIFF_MAX_Z).toBe(-0.1)
    expect(OBSTACLE_MIN_Z).toBe(0.1)
    expect(OBSTACLE_MAX_Z).toBe(0.6)
  })
  it('colours by height band: cliff, obstacle, other', () => {
    const c = depthColors(new Float32Array([0, 0, CLIFF_MAX_Z - 0.01, 0, 0, OBSTACLE_MIN_Z + 0.01, 0, 0, OBSTACLE_MAX_Z + 0.2, 0, 0, 0]))
    const rgb = (i: number) => Array.from(c.slice(i * 3, i * 3 + 3)).map((v) => Math.round(v * 1000))
    expect(c.length).toBe(12)
    expect(new Set([rgb(0).join(), rgb(1).join(), rgb(2).join()]).size).toBe(3)
    expect(rgb(3)).toEqual(rgb(2))
  })
  it('lidar uses one colour', () => {
    expect(LIDAR_COLOR).toMatch(/^#[0-9a-f]{6}$/i)
  })
})

describe('rgbdCloudToMap', () => {
  function b64(bytes: Uint8Array): string {
    let s = ''
    bytes.forEach((b) => (s += String.fromCharCode(b)))
    return btoa(s)
  }
  function rgbd(cm: number[], rgb: number[] | null, pose: unknown = YAW90) {
    return {
      frame: 'head_camera_optical_frame',
      frame_id: 'head_camera_optical_frame',
      stamp: 3,
      count: cm.length / 3,
      voxel_m: 0.02,
      source_count: cm.length / 3,
      dropped_out_of_range: 0,
      points_b64: b64(new Uint8Array(new Int16Array(cm).buffer)),
      ...(rgb ? { colors_b64: b64(new Uint8Array(rgb)) } : {}),
      frame_pose: pose,
    }
  }

  it('places the decoded sensor-frame points in the map frame with their colours', () => {
    const out = rgbdCloudToMap(rgbd([100, 0, 0, 0, 100, 0], [255, 0, 0, 0, 255, 0]))
    expect(out).not.toBeNull()
    expect(out!.count).toBe(2)
    // yaw +90 deg: sensor x -> map y, sensor y -> map -x; then the pose translation (1, 2, 0.5).
    const xyz = Array.from(out!.positions)
    expect(xyz[0]).toBeCloseTo(1)
    expect(xyz[1]).toBeCloseTo(3)
    expect(xyz[2]).toBeCloseTo(0.5)
    expect(xyz[3]).toBeCloseTo(0)
    expect(xyz[4]).toBeCloseTo(2)
    expect(Array.from(out!.colors!)).toEqual([1, 0, 0, 0, 1, 0])
  })

  it('draws nothing without a finite frame_pose or with a malformed payload', () => {
    expect(rgbdCloudToMap(rgbd([1, 2, 3], null, null))).toBeNull()
    expect(rgbdCloudToMap(rgbd([1, 2, 3], null, { ...YAW90, qw: NaN }))).toBeNull()
    expect(rgbdCloudToMap({ frame_pose: YAW90 })).toBeNull()
    expect(rgbdCloudToMap(null)).toBeNull()
  })

  it('has no colours when the payload carries none', () => {
    expect(rgbdCloudToMap(rgbd([1, 2, 3], null))!.colors).toBeUndefined()
  })
})
