// @vitest-environment node
import { describe, expect, it } from 'vitest'
import {
  centerOn,
  DRAG_THRESHOLD_PX,
  fitView,
  goalFromGesture,
  MapMeta,
  mapImageTransform,
  mapPixelToWorld,
  MAX_SCALE,
  MIN_SCALE,
  niceScaleBar,
  normalizeAngle,
  panView,
  pinchView,
  quaternionToYaw,
  screenToWorld,
  View,
  worldToMapPixel,
  worldToScreen,
  yawToQuaternion,
  zoomAboutPoint,
} from './mapMath'

const view: View = { scale: 20, offsetX: 100, offsetY: 200 }
const meta: MapMeta = { width: 40, height: 20, resolution: 0.05, origin: { x: -1, y: -0.5, yaw: 0 } }

function close(a: { x: number; y: number }, b: { x: number; y: number }, digits = 9) {
  expect(a.x).toBeCloseTo(b.x, digits)
  expect(a.y).toBeCloseTo(b.y, digits)
}

describe('world <-> screen', () => {
  it('maps world y up to screen y down', () => {
    expect(worldToScreen(view, { x: 0, y: 0 })).toEqual({ x: 100, y: 200 })
    expect(worldToScreen(view, { x: 1, y: 1 })).toEqual({ x: 120, y: 180 })
  })

  it('round-trips', () => {
    const p = { x: -3.7, y: 2.25 }
    close(screenToWorld(view, worldToScreen(view, p)), p)
  })
})

describe('map pixel <-> world', () => {
  it('places image top-left at origin + height (rows flipped, y up)', () => {
    close(mapPixelToWorld(meta, 0, 0), { x: -1, y: 0.5 })
    close(mapPixelToWorld(meta, 0, 20), { x: -1, y: -0.5 })
    close(mapPixelToWorld(meta, 40, 20), { x: 1, y: -0.5 })
  })

  it('round-trips with yaw', () => {
    const rotated: MapMeta = { ...meta, origin: { x: 2, y: 3, yaw: 0.7 } }
    const px = worldToMapPixel(rotated, mapPixelToWorld(rotated, 12.5, 7.25))
    expect(px.u).toBeCloseTo(12.5, 9)
    expect(px.v).toBeCloseTo(7.25, 9)
  })

  it('applies origin yaw', () => {
    const rotated: MapMeta = { ...meta, origin: { x: 0, y: 0, yaw: Math.PI / 2 } }
    // grid cell column axis (u) points along world +y when yaw = 90 deg
    close(mapPixelToWorld(rotated, 20, 20), { x: 0, y: 1 })
  })

  it('canvas image transform matches pixel -> world -> screen', () => {
    const rotated: MapMeta = { ...meta, origin: { x: 0.3, y: -2, yaw: -1.1 } }
    const [a, b, c, d, e, f] = mapImageTransform(rotated, view)
    for (const [u, v] of [[0, 0], [40, 0], [13, 17]]) {
      const expected = worldToScreen(view, mapPixelToWorld(rotated, u, v))
      close({ x: a * u + c * v + e, y: b * u + d * v + f }, expected)
    }
  })
})

describe('pan / zoom', () => {
  it('pans by screen delta', () => {
    expect(panView(view, 5, -7)).toEqual({ scale: 20, offsetX: 105, offsetY: 193 })
  })

  it('zoom keeps the world point under the cursor fixed', () => {
    const cursor = { x: 333, y: 44 }
    const before = screenToWorld(view, cursor)
    const zoomed = zoomAboutPoint(view, cursor, 1.7)
    expect(zoomed.scale).toBeCloseTo(34)
    close(screenToWorld(zoomed, cursor), before)
  })

  it('clamps scale', () => {
    expect(zoomAboutPoint(view, { x: 0, y: 0 }, 1e9).scale).toBe(MAX_SCALE)
    expect(zoomAboutPoint(view, { x: 0, y: 0 }, 1e-9).scale).toBe(MIN_SCALE)
  })

  it('pinch zooms by finger distance ratio and pans with the midpoint', () => {
    const prev: [{ x: number; y: number }, { x: number; y: number }] = [{ x: 100, y: 100 }, { x: 200, y: 100 }]
    const next: [{ x: number; y: number }, { x: number; y: number }] = [{ x: 60, y: 120 }, { x: 260, y: 120 }]
    const worldAtPrevMid = screenToWorld(view, { x: 150, y: 100 })
    const v = pinchView(view, prev, next)
    expect(v.scale).toBeCloseTo(40)
    close(worldToScreen(v, worldAtPrevMid), { x: 160, y: 120 })
  })

  it('fits the map in the canvas, centred', () => {
    const v = fitView(meta, 400, 400, 1)
    // map is 2 m x 1 m; width limits: 400 px / 2 m
    expect(v.scale).toBeCloseTo(200)
    close(worldToScreen(v, { x: 0, y: 0 }), { x: 200, y: 200 })
  })

  it('centres on a world point', () => {
    const v = centerOn(view, { x: 3, y: -1 }, 640, 480)
    expect(v.scale).toBe(view.scale)
    close(worldToScreen(v, { x: 3, y: -1 }), { x: 320, y: 240 })
  })
})

describe('quaternion / yaw', () => {
  it('round-trips yaw', () => {
    for (const yaw of [0, 0.5, -2.9, Math.PI / 2]) {
      expect(quaternionToYaw(yawToQuaternion(yaw))).toBeCloseTo(yaw, 9)
    }
  })

  it('builds a planar unit quaternion', () => {
    const q = yawToQuaternion(Math.PI)
    expect(q.x).toBe(0)
    expect(q.y).toBe(0)
    expect(q.z).toBeCloseTo(1)
    expect(q.w).toBeCloseTo(0)
  })

  it('normalizes angles into (-pi, pi]', () => {
    expect(normalizeAngle(3 * Math.PI)).toBeCloseTo(Math.PI)
    expect(normalizeAngle(-3 * Math.PI / 2)).toBeCloseTo(Math.PI / 2)
  })
})

describe('goal from gesture', () => {
  const press = { x: 1, y: 1 }

  it('drag sets heading from press to release', () => {
    const g = goalFromGesture(press, { x: 1, y: 3 }, DRAG_THRESHOLD_PX + 1, { x: 0, y: 0, yaw: 0 })
    expect(g.x).toBe(1)
    expect(g.y).toBe(1)
    expect(g.yaw).toBeCloseTo(Math.PI / 2)
  })

  it('plain click faces from robot to goal', () => {
    const g = goalFromGesture(press, { x: 1.001, y: 1 }, DRAG_THRESHOLD_PX - 1, { x: 2, y: 1, yaw: 0.3 })
    expect(g.x).toBe(1)
    expect(g.y).toBe(1)
    expect(g.yaw).toBeCloseTo(Math.PI)
  })

  it('plain click without robot pose keeps heading 0', () => {
    expect(goalFromGesture(press, press, 0, null).yaw).toBe(0)
  })

  it('plain click on the robot keeps the robot heading', () => {
    expect(goalFromGesture(press, press, 0, { x: 1, y: 1, yaw: 0.8 }).yaw).toBeCloseTo(0.8)
  })
})

describe('scale bar', () => {
  it('picks a 1-2-5 length close to the target width', () => {
    expect(niceScaleBar(20, 100)).toEqual({ meters: 5, px: 100 })
    const bar = niceScaleBar(37, 100)
    expect([1, 2, 5]).toContain(bar.meters)
    expect(bar.px).toBeCloseTo(bar.meters * 37)
    expect(bar.px).toBeLessThanOrEqual(100)
  })

  it('handles sub-metre scales', () => {
    expect(niceScaleBar(2000, 100)).toEqual({ meters: 0.05, px: 100 })
  })
})
