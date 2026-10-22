import { describe, expect, it, vi } from 'vitest'
import {
  decodeGroundCloud,
  GROUND_COLORS,
  GROUND_EDGE_LEGEND,
  warningAreaColour,
  warningAreaReadouts,
  groundReadout,
  groundVisible,
  labelColour,
  planePatch,
  regionEdges,
  warningPolygons,
  type GroundState,
} from './groundLayer'

const IDENTITY = { x: 0, y: 0, z: 0, qx: 0, qy: 0, qz: 0, qw: 1 }

vi.mock('../logging', () => ({ default: { warn: vi.fn(), info: vi.fn(), debug: vi.fn() } }))

function b64(bytes: number[]): string {
  return btoa(String.fromCharCode(...bytes))
}

function int16(values: number[]): number[] {
  const out: number[] = []
  for (const v of values) out.push(v & 0xff, (v >> 8) & 0xff)
  return out
}

function state(over: Partial<GroundState> = {}): GroundState {
  return {
    stamp_ns: 1,
    frame: 'base_link',
    prior: null,
    imu_ok: true,
    plane: { normal: [0, 0, 1], height_m: 0, inliers: 100, inlier_ratio: 0.9, source: 'fit', age_s: 0 },
    reason: null,
    slope_deg: 1.5,
    slope_roll_deg: 1,
    slope_pitch_deg: 1,
    slope_level: 'ok',
    base_tilt_deg: 0,
    plane_vs_base_deg: 0,
    traversability: null,
    obstacle_points: 0,
    drop_points: 0,
    regions: [],
    warnings: [],
    colour: null,
    ...over,
  } as GroundState
}

describe('decodeGroundCloud', () => {
  it('decodes points, colours and labels and places them by frame_pose', () => {
    const payload = {
      frame_id: 'base_link',
      count: 2,
      voxel_m: 0.02,
      points_b64: b64(int16([100, 0, 0, 0, 200, 50])),
      colors_b64: b64([255, 0, 0, 0, 255, 0]),
      labels_b64: b64([0, 1]),
      frame_pose: { ...IDENTITY, x: 1 },
    }
    const out = decodeGroundCloud(payload)
    expect(out).not.toBeNull()
    expect(out!.count).toBe(2)
    expect(Array.from(out!.positions)).toEqual([2, 0, 0, 1, 2, 0.5])
    expect(Array.from(out!.labels)).toEqual([0, 1])
    expect(out!.colors![0]).toBe(1)
    expect(out!.labelColors.length).toBe(6)
  })

  it('rejects labels whose length does not match the count, a missing pose and non-objects', () => {
    const base = { frame_id: 'base_link', count: 2, voxel_m: 0.02, points_b64: b64(int16([0, 0, 0, 0, 0, 0])), frame_pose: IDENTITY }
    expect(decodeGroundCloud({ ...base, labels_b64: b64([0]) })).toBeNull()
    expect(decodeGroundCloud({ ...base, labels_b64: b64([0, 0]), frame_pose: undefined })).toBeNull()
    expect(decodeGroundCloud(null)).toBeNull()
  })

  it('treats a payload without labels as unlabelled', () => {
    const out = decodeGroundCloud({ frame_id: 'base_link', count: 1, voxel_m: 0.02, points_b64: b64(int16([0, 0, 0])), frame_pose: IDENTITY })
    expect(Array.from(out!.labels)).toEqual([4])
  })
})

describe('labelColour', () => {
  it('gives floor green, obstacle red, drop blue, mismatch orange and unlabelled or unknown grey', () => {
    expect(labelColour(0)).toBe(GROUND_COLORS.floor)
    expect(labelColour(1)).toBe(GROUND_COLORS.obstacle)
    expect(labelColour(2)).toBe(GROUND_COLORS.drop)
    expect(labelColour(3)).toBe(GROUND_COLORS.mismatch)
    expect(labelColour(4)).toBe(GROUND_COLORS.unlabelled)
    expect(labelColour(99)).toBe(GROUND_COLORS.unlabelled)
    expect(new Set([0, 1, 2, 3, 4].map(labelColour)).size).toBe(5)
  })
})

describe('planePatch', () => {
  it('is a 2 m square at the plane height for a level plane', () => {
    const corners = planePatch(state({ plane: { ...state().plane!, height_m: -0.02 } }))!
    expect(corners).toHaveLength(4)
    for (const c of corners) {
      expect(Math.abs(c[0])).toBeCloseTo(1)
      expect(Math.abs(c[1])).toBeCloseTo(1)
      expect(c[2]).toBeCloseTo(-0.02)
    }
  })

  it('tilts with the normal: a 5 deg pitched plane rises 1 m * tan(5 deg) over one metre', () => {
    const a = (5 * Math.PI) / 180
    const corners = planePatch(state({ plane: { ...state().plane!, normal: [-Math.sin(a), 0, Math.cos(a)], height_m: 0 } }))!
    for (const c of corners) {
      expect(c[2]).toBeCloseTo(c[0] * Math.tan(a))
    }
    expect(Math.max(...corners.map((c) => c[2]))).toBeCloseTo(Math.tan(a))
  })

  it('is null without a plane or with a vertical normal', () => {
    expect(planePatch(state({ plane: null }))).toBeNull()
    expect(planePatch(state({ plane: { ...state().plane!, normal: [1, 0, 0] } }))).toBeNull()
  })
})

describe('groundVisible', () => {
  it('is false for absent, stale and plane-null states and true for a fresh plane', () => {
    expect(groundVisible(undefined)).toBe(false)
    expect(groundVisible(null)).toBe(false)
    expect(groundVisible({ ...state(), stale: true })).toBe(false)
    expect(groundVisible(state({ plane: null }))).toBe(false)
    expect(groundVisible(state())).toBe(true)
  })
})

describe('regionEdges', () => {
  const regions = [
    { name: 'step_0', kind: 'step', class: 'obstacle', height_m: 0.05, frame: 'base_link', edge: { point: [0.5, 0.2], direction: [0, 1], side: 'left' }, distance_m: 0.5, points: 20 },
    { name: 'drop_0', kind: 'drop', class: 'warning', height_m: -0.02, frame: 'base_link', edge: { point: [1, 0], direction: [0.6, 0.8], side: 'right' }, distance_m: 1, points: 12 },
  ]

  it('gives 2 m segments centred on the edge point along the direction, red for obstacle and yellow for warning', () => {
    const edges = regionEdges(state({ regions } as Partial<GroundState>))
    expect(edges).toHaveLength(2)
    expect(edges[0].a).toEqual([0.5, -0.8])
    expect(edges[0].b).toEqual([0.5, 1.2])
    expect(edges[0].colour).toBe(GROUND_COLORS.edgeObstacle)
    expect(edges[0].kind).toBe('step')
    expect(edges[1].colour).toBe(GROUND_COLORS.edgeWarning)
    expect(Math.hypot(edges[1].b[0] - edges[1].a[0], edges[1].b[1] - edges[1].a[1])).toBeCloseTo(2)
    expect((edges[1].a[0] + edges[1].b[0]) / 2).toBeCloseTo(1)
  })

  it('is empty for an invisible state', () => {
    expect(regionEdges(state({ plane: null, regions } as Partial<GroundState>))).toEqual([])
    expect(regionEdges({ ...state({ regions } as Partial<GroundState>), stale: true })).toEqual([])
    expect(regionEdges(undefined)).toEqual([])
  })
})

describe('warningPolygons', () => {
  const warnings = [{ kind: 'step', polygon: [[0, 0], [1, 0], [1, 1]], height_m: 0.03, slope_deg: 2, source: 'step_0' }]

  it('returns the polygons with kind and source', () => {
    expect(warningPolygons(state({ warnings } as Partial<GroundState>))).toEqual([
      { kind: 'step', source: 'step_0', polygon: [[0, 0], [1, 0], [1, 1]] },
    ])
  })

  it('skips polygons with fewer than three vertices and invisible states', () => {
    expect(warningPolygons(state({ warnings: [{ ...warnings[0], polygon: [[0, 0], [1, 0]] }] } as Partial<GroundState>))).toEqual([])
    expect(warningPolygons({ ...state({ warnings } as Partial<GroundState>), stale: true })).toEqual([])
  })
})

describe('groundReadout', () => {
  it('reads slope, level and plane height, and is null for an invisible state', () => {
    const r = groundReadout(state({ slope_deg: 6.25, slope_level: 'warning', plane: { ...state().plane!, height_m: -0.031 } }))
    expect(r).toEqual({ slope: '6.3 deg', level: 'warning', height: '-0.031 m' })
    expect(groundReadout(undefined)).toBeNull()
    expect(groundReadout(state({ plane: null }))).toBeNull()
  })
})

describe('no_return areas', () => {
  const noReturn = { kind: 'no_return', polygon: [[0, 0], [1, 0], [1, 1]], height_m: null, slope_deg: null, source: 'no_return_0' }
  const step = { kind: 'step', polygon: [[0, 0], [1, 0], [1, 1]], height_m: 0.03, slope_deg: 2, source: 'step_0' }

  it('draws no_return areas in their own colour with a legend entry', () => {
    expect(GROUND_COLORS.noReturnFill).not.toBe(GROUND_COLORS.warningFill)
    expect(GROUND_COLORS.noReturnFill).not.toBe(GROUND_COLORS.mismatch)
    expect(warningAreaColour('no_return')).toBe(GROUND_COLORS.noReturnFill)
    expect(warningAreaColour('step')).toBe(GROUND_COLORS.warningFill)
    expect(GROUND_EDGE_LEGEND).toContainEqual(['No ToF return (caution)', GROUND_COLORS.noReturnFill])
    const polys = warningPolygons(state({ warnings: [noReturn, step] } as unknown as Partial<GroundState>))
    expect(polys.map((w) => warningAreaColour(w.kind))).toEqual([GROUND_COLORS.noReturnFill, GROUND_COLORS.warningFill])
  })

  it('reads null area heights and slopes as n/a', () => {
    expect(warningAreaReadouts(state({ warnings: [noReturn, step] } as unknown as Partial<GroundState>))).toEqual([
      { source: 'no_return_0', kind: 'no_return', height: 'n/a', slope: 'n/a' },
      { source: 'step_0', kind: 'step', height: '0.030 m', slope: '2.0 deg' },
    ])
    expect(warningAreaReadouts(undefined)).toEqual([])
  })
})
