// Pure coordinate math for the map_nav tab: world (metres, y up) <-> map image pixel <-> screen (CSS px, y down).

export const MIN_SCALE = 2 // px per metre
export const MAX_SCALE = 2000 // px per metre
export const DRAG_THRESHOLD_PX = 8 // a press that moves less than this is a plain click

export interface Vec2 {
  x: number
  y: number
}

export interface Pose2D {
  x: number
  y: number
  yaw: number
}

/** Screen = (x * scale + offsetX, -y * scale + offsetY); scale in px per metre. */
export interface View {
  scale: number
  offsetX: number
  offsetY: number
}

/** Placement of an OccupancyGrid image (as serialized by the backend). */
export interface MapMeta {
  width: number
  height: number
  resolution: number
  origin: Pose2D
}

export interface Quaternion {
  x: number
  y: number
  z: number
  w: number
}

export function worldToScreen(view: View, p: Vec2): Vec2 {
  return { x: p.x * view.scale + view.offsetX, y: -p.y * view.scale + view.offsetY }
}

export function screenToWorld(view: View, s: Vec2): Vec2 {
  return { x: (s.x - view.offsetX) / view.scale, y: -(s.y - view.offsetY) / view.scale }
}

/**
 * World position of map image pixel corner (u, v). Image row 0 is the top (highest y) because the backend
 * flips grid rows, so grid coordinates are (u, height - v) cells from the origin, rotated by origin yaw.
 */
export function mapPixelToWorld(meta: MapMeta, u: number, v: number): Vec2 {
  const gx = u * meta.resolution
  const gy = (meta.height - v) * meta.resolution
  const c = Math.cos(meta.origin.yaw)
  const s = Math.sin(meta.origin.yaw)
  return { x: meta.origin.x + c * gx - s * gy, y: meta.origin.y + s * gx + c * gy }
}

export function worldToMapPixel(meta: MapMeta, p: Vec2): { u: number; v: number } {
  const dx = p.x - meta.origin.x
  const dy = p.y - meta.origin.y
  const c = Math.cos(meta.origin.yaw)
  const s = Math.sin(meta.origin.yaw)
  const gx = c * dx + s * dy
  const gy = -s * dx + c * dy
  return { u: gx / meta.resolution, v: meta.height - gy / meta.resolution }
}

/** Canvas setTransform(a, b, c, d, e, f) that draws the map image (in pixel units) at its world placement. */
export function mapImageTransform(meta: MapMeta, view: View): [number, number, number, number, number, number] {
  const k = view.scale * meta.resolution
  const c = Math.cos(meta.origin.yaw)
  const s = Math.sin(meta.origin.yaw)
  const top = worldToScreen(view, mapPixelToWorld(meta, 0, 0))
  return [k * c, -k * s, k * s, k * c, top.x, top.y]
}

export function panView(view: View, dx: number, dy: number): View {
  return { ...view, offsetX: view.offsetX + dx, offsetY: view.offsetY + dy }
}

/** Multiply the scale by factor (clamped) keeping the world point under screen point s fixed. */
export function zoomAboutPoint(
  view: View,
  s: Vec2,
  factor: number,
  minScale: number = MIN_SCALE,
  maxScale: number = MAX_SCALE,
): View {
  const scale = Math.min(maxScale, Math.max(minScale, view.scale * factor))
  const w = screenToWorld(view, s)
  return { scale, offsetX: s.x - w.x * scale, offsetY: s.y + w.y * scale }
}

/** Two-finger gesture: zoom by the finger distance ratio about the old midpoint, then follow the midpoint. */
export function pinchView(view: View, prev: [Vec2, Vec2], next: [Vec2, Vec2]): View {
  const dist = (a: Vec2, b: Vec2) => Math.hypot(a.x - b.x, a.y - b.y)
  const mid = (a: Vec2, b: Vec2) => ({ x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 })
  const d0 = dist(prev[0], prev[1])
  const factor = d0 > 0 ? dist(next[0], next[1]) / d0 : 1
  const m0 = mid(prev[0], prev[1])
  const m1 = mid(next[0], next[1])
  return panView(zoomAboutPoint(view, m0, factor), m1.x - m0.x, m1.y - m0.y)
}

/** View that shows the whole map centred in a width x height canvas, using `fill` of the space. */
export function fitView(meta: MapMeta, width: number, height: number, fill = 0.95): View {
  const corners = [
    mapPixelToWorld(meta, 0, 0),
    mapPixelToWorld(meta, meta.width, 0),
    mapPixelToWorld(meta, 0, meta.height),
    mapPixelToWorld(meta, meta.width, meta.height),
  ]
  const xs = corners.map((p) => p.x)
  const ys = corners.map((p) => p.y)
  const spanX = Math.max(...xs) - Math.min(...xs)
  const spanY = Math.max(...ys) - Math.min(...ys)
  const raw = Math.min(width / Math.max(spanX, 1e-6), height / Math.max(spanY, 1e-6)) * fill
  const scale = Math.min(MAX_SCALE, Math.max(MIN_SCALE, raw))
  const centre = { x: (Math.max(...xs) + Math.min(...xs)) / 2, y: (Math.max(...ys) + Math.min(...ys)) / 2 }
  return centerOn({ scale, offsetX: 0, offsetY: 0 }, centre, width, height)
}

/** Keep the scale, move the view so world point p sits at the canvas centre. */
export function centerOn(view: View, p: Vec2, width: number, height: number): View {
  return { scale: view.scale, offsetX: width / 2 - p.x * view.scale, offsetY: height / 2 + p.y * view.scale }
}

export function normalizeAngle(a: number): number {
  let r = a % (2 * Math.PI)
  if (r <= -Math.PI) r += 2 * Math.PI
  if (r > Math.PI) r -= 2 * Math.PI
  return r
}

export function quaternionToYaw(q: Quaternion): number {
  return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
}

export function yawToQuaternion(yaw: number): Quaternion {
  return { x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) }
}

/**
 * Goal pose from a press/release gesture (world coordinates). The press point is the position. A drag longer than
 * thresholdPx (screen distance) sets the heading along the drag; a plain click faces from the robot to the goal,
 * keeps the robot heading when clicking on the robot, and is 0 when the robot pose is unknown.
 */
export function goalFromGesture(
  press: Vec2,
  release: Vec2,
  dragPx: number,
  robot: Pose2D | null,
  thresholdPx: number = DRAG_THRESHOLD_PX,
): Pose2D {
  if (dragPx >= thresholdPx) {
    return { x: press.x, y: press.y, yaw: Math.atan2(release.y - press.y, release.x - press.x) }
  }
  if (!robot) return { x: press.x, y: press.y, yaw: 0 }
  const dx = press.x - robot.x
  const dy = press.y - robot.y
  if (Math.hypot(dx, dy) < 1e-6) return { x: press.x, y: press.y, yaw: robot.yaw }
  return { x: press.x, y: press.y, yaw: Math.atan2(dy, dx) }
}

/** Largest 1-2-5 x 10^n metre length whose on-screen width does not exceed targetPx. */
export function niceScaleBar(scale: number, targetPx = 100): { meters: number; px: number } {
  const maxMeters = targetPx / scale
  const exp = Math.floor(Math.log10(maxMeters))
  let meters = Math.pow(10, exp)
  for (const m of [1, 2, 5, 10]) {
    const candidate = Number((m * Math.pow(10, exp)).toPrecision(6))
    if (candidate <= maxMeters + 1e-9) meters = candidate
  }
  return { meters, px: meters * scale }
}

/**
 * Index i of the polygon edge (points[i] -> points[i+1], wrapping) that faces the robot heading: the edge whose
 * midpoint lies furthest ahead of the robot along its yaw. Used to highlight the front of the footprint.
 * Returns -1 for fewer than two points.
 */
export function frontEdgeIndex(points: [number, number][], pose: Pose2D): number {
  if (points.length < 2) return -1
  const hx = Math.cos(pose.yaw)
  const hy = Math.sin(pose.yaw)
  let best = -1
  let bestAhead = -Infinity
  for (let i = 0; i < points.length; i++) {
    const [ax, ay] = points[i]
    const [bx, by] = points[(i + 1) % points.length]
    const ahead = ((ax + bx) / 2 - pose.x) * hx + ((ay + by) / 2 - pose.y) * hy
    if (ahead > bestAhead) {
      bestAhead = ahead
      best = i
    }
  }
  return best
}
