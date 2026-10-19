import { describe, expect, it } from 'vitest'
import { buildLidarWalls, measuredColumnsFromCloud, type WallGrid } from './lidarWalls'

const FREE = 254
const OCC = 0
const UNK = 205

/** Image rows top first, as the backend PNG: rows[0] is the northmost row. */
function grid(rows: number[][], over: Partial<WallGrid> = {}): WallGrid {
  return {
    width: rows[0].length,
    height: rows.length,
    resolution: 0.5,
    origin: { x: 10, y: 20, yaw: 0 },
    cells: Uint8Array.from(rows.flat()),
    ...over,
  }
}

describe('buildLidarWalls', () => {
  it('turns an occupied run into one box of the configured height', () => {
    const walls = buildLidarWalls(grid([[FREE, OCC, OCC, OCC, FREE]]), new Set(), 1.5)
    expect(walls).toHaveLength(1)
    const w = walls[0]
    expect(w.height).toBe(1.5)
    expect(w.length).toBeCloseTo(1.5)
    expect(w.width).toBeCloseTo(0.5)
    expect(w.center.x).toBeCloseTo(10 + 1.25)
    expect(w.center.y).toBeCloseTo(20 + 0.25)
    expect(w.yaw).toBe(0)
  })

  it('gives nothing for free and unknown cells', () => {
    expect(buildLidarWalls(grid([[FREE, UNK, FREE], [UNK, UNK, FREE]]), new Set(), 1)).toEqual([])
  })

  it('splits runs per row and at gaps', () => {
    const walls = buildLidarWalls(grid([[OCC, FREE, OCC], [OCC, OCC, OCC]]), new Set(), 1)
    expect(walls).toHaveLength(3)
  })

  it('skips columns holding a measured point and splits the run there', () => {
    const g = grid([[OCC, OCC, OCC]])
    const measured = measuredColumnsFromCloud(new Float32Array([10 + 0.75, 1.2, -(20 + 0.25)]), 0.5, g.origin)
    const walls = buildLidarWalls(g, measured, 1)
    expect(walls).toHaveLength(2)
    expect(walls.map((w) => w.length)).toEqual([0.5, 0.5])
  })

  it('hashes cloud x/y (three order: x, up, -y) into map cells', () => {
    const measured = measuredColumnsFromCloud(new Float32Array([0.1, 5, -0.1, 0.1, 9, -0.1]), 0.5, { x: 0, y: 0, yaw: 0 })
    expect(measured.size).toBe(1)
  })

  it('rotates boxes by the origin yaw', () => {
    const w = buildLidarWalls(grid([[OCC, OCC]], { origin: { x: 0, y: 0, yaw: Math.PI / 2 } }), new Set(), 1)[0]
    expect(w.yaw).toBeCloseTo(Math.PI / 2)
    expect(w.center.x).toBeCloseTo(-0.25)
    expect(w.center.y).toBeCloseTo(0.5)
  })
})
