/**
 * Lidar-derived walls (design D13): occupied cells of the SLAM /map extruded to a fixed height, except where the
 * measured 3D cloud already has a point in that cell column. Pure geometry, no three.js.
 */
import type { Pose2D } from '../map/mapMath'

/** Grey level at or below which an image cell is occupied (the SLAM map layer draws occupied as black, 0). */
export const OCCUPIED_GREY_MAX = 64

/** SLAM map as the browser holds it: grey cells in image order (row 0 is the northmost row), plus placement. */
export interface WallGrid {
  width: number
  height: number
  resolution: number
  origin: Pose2D
  cells: ArrayLike<number>
}

/** One extruded wall box in the map frame; z runs from 0 to height. length is along the grid x axis, width along y. */
export interface WallBox {
  center: { x: number; y: number }
  length: number
  width: number
  height: number
  yaw: number
}

/**
 * Key of one grid cell column.
 *
 * @param col - cell index along the grid x axis
 * @param row - cell index along the grid y axis, counted from the bottom (origin) row
 * @returns a hashable key
 */
export function columnKey(col: number, row: number): string {
  return `${col},${row}`
}

/**
 * Cell columns that contain a measured 3D point.
 *
 * @param positions - cloud positions in three.js order (x, up, -y), metres in the map frame
 * @param resolution - map resolution in metres per cell
 * @param origin - map origin pose
 * @returns the set of columnKey values
 */
export function measuredColumnsFromCloud(positions: ArrayLike<number>, resolution: number, origin: Pose2D): Set<string> {
  const c = Math.cos(origin.yaw)
  const s = Math.sin(origin.yaw)
  const out = new Set<string>()
  for (let i = 0; i + 2 < positions.length; i += 3) {
    const dx = positions[i] - origin.x
    const dy = 0 - positions[i + 2] - origin.y
    out.add(columnKey(Math.floor((c * dx + s * dy) / resolution), Math.floor((-s * dx + c * dy) / resolution)))
  }
  return out
}

/**
 * Build wall boxes: one per run of consecutive occupied cells in a row, split where a column holds a measured point.
 *
 * @param grid - the SLAM map cells and placement
 * @param measuredColumns - columns to skip, see measuredColumnsFromCloud
 * @param heightM - extrusion height in metres (lidar_wall_height_m)
 * @returns boxes in the map frame
 */
export function buildLidarWalls(grid: WallGrid, measuredColumns: ReadonlySet<string>, heightM: number): WallBox[] {
  const { width, height, resolution: res, origin } = grid
  const c = Math.cos(origin.yaw)
  const s = Math.sin(origin.yaw)
  const boxes: WallBox[] = []
  for (let v = 0; v < height; v++) {
    const row = height - 1 - v
    let start = -1
    const close = (end: number) => {
      if (start < 0) return
      const gx = ((start + end) / 2) * res
      const gy = (row + 0.5) * res
      boxes.push({
        center: { x: origin.x + c * gx - s * gy, y: origin.y + s * gx + c * gy },
        length: (end - start) * res,
        width: res,
        height: heightM,
        yaw: origin.yaw,
      })
      start = -1
    }
    for (let u = 0; u < width; u++) {
      const wall = grid.cells[v * width + u] <= OCCUPIED_GREY_MAX && !measuredColumns.has(columnKey(u, row))
      if (wall && start < 0) start = u
      else if (!wall) close(u)
    }
    close(width)
  }
  return boxes
}
