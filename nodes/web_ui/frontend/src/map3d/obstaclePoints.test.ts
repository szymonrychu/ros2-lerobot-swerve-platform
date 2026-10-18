import { describe, expect, it } from 'vitest'
import {
  CLIFF_MAX_Z,
  LIDAR_COLOR,
  OBSTACLE_MAX_Z,
  OBSTACLE_MIN_Z,
  cloudToMapPoints,
  parsePointsPayload,
  scanToMapPoints,
  tofColors,
  type PointsPayload,
} from './obstaclePoints'

const IDENTITY = { x: 0, y: 0, z: 0, qx: 0, qy: 0, qz: 0, qw: 1 }
const YAW90 = { x: 1, y: 2, z: 0.5, qx: 0, qy: 0, qz: Math.SQRT1_2, qw: Math.SQRT1_2 }

function payload(points: number[], pose = IDENTITY): PointsPayload {
  return { stamp: 1, frame: 'tof', points, frame_pose: pose }
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

describe('tofColors', () => {
  it('colours by height band: cliff, obstacle, other', () => {
    const c = tofColors(new Float32Array([0, 0, CLIFF_MAX_Z - 0.01, 0, 0, OBSTACLE_MIN_Z + 0.01, 0, 0, OBSTACLE_MAX_Z + 0.2, 0, 0, 0]))
    const rgb = (i: number) => Array.from(c.slice(i * 3, i * 3 + 3)).map((v) => Math.round(v * 1000))
    expect(c.length).toBe(12)
    expect(new Set([rgb(0).join(), rgb(1).join(), rgb(2).join()]).size).toBe(3)
    expect(rgb(3)).toEqual(rgb(2))
  })
  it('lidar uses one colour', () => {
    expect(LIDAR_COLOR).toMatch(/^#[0-9a-f]{6}$/i)
  })
})
