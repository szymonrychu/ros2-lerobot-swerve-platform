import { beforeEach, describe, expect, it, vi } from 'vitest'
import type { Map3dCloudPayload } from '../types'
import log from '../logging'
import { decodeMap3dCloud } from './cloudDecode'

vi.mock('../logging', () => ({ default: { warn: vi.fn(), info: vi.fn(), debug: vi.fn() } }))

function b64(bytes: Uint8Array): string {
  let s = ''
  bytes.forEach((b) => (s += String.fromCharCode(b)))
  return btoa(s)
}

function build(cm: number[], rgb?: number[], over: Partial<Map3dCloudPayload> = {}): Map3dCloudPayload {
  const pts = new Int16Array(cm)
  return {
    frame_id: 'rtabmap_map',
    stamp: 1,
    count: cm.length / 3,
    voxel_m: 0.05,
    source_count: cm.length / 3,
    dropped_out_of_range: 0,
    points_b64: b64(new Uint8Array(pts.buffer)),
    ...(rgb ? { colors_b64: b64(new Uint8Array(rgb)) } : {}),
    ...over,
  }
}

describe('decodeMap3dCloud', () => {
  it('round-trips points into three.js order in metres, with colours', () => {
    const out = decodeMap3dCloud(build([100, 200, 50, -100, 0, 300], [255, 0, 0, 0, 255, 51]), 'rtabmap_map')
    expect(out).not.toBeNull()
    expect(out!.count).toBe(2)
    expect(out!.voxel).toBe(0.05)
    expect(Array.from(out!.positions)).toEqual([1, 0.5, -2, -1, 3, 0].map((v) => Math.fround(v)))
    expect(Array.from(out!.colors!)).toEqual([1, 0, 0, 0, 1, 51 / 255].map((v) => Math.fround(v)))
  })

  it('has no colours when the payload carries none, and handles an empty cloud', () => {
    expect(decodeMap3dCloud(build([1, 2, 3]), 'rtabmap_map')!.colors).toBeUndefined()
    const empty = decodeMap3dCloud({ ...build([]), points_b64: '' }, 'rtabmap_map')
    expect(empty!.count).toBe(0)
  })

  it('rejects a frame other than the tab map3d_frame_id', () => {
    expect(decodeMap3dCloud(build([1, 2, 3], undefined, { frame_id: 'map' }), 'rtabmap_map')).toBeNull()
  })

  it('rejects byte lengths that do not match count', () => {
    expect(decodeMap3dCloud(build([1, 2, 3, 4, 5, 6], undefined, { count: 3 }), 'rtabmap_map')).toBeNull()
    expect(decodeMap3dCloud(build([1, 2, 3], [1, 2], {}), 'rtabmap_map')).toBeNull()
  })

  it('rejects non-payloads', () => {
    expect(decodeMap3dCloud(null, 'rtabmap_map')).toBeNull()
    expect(decodeMap3dCloud({ count: 1 }, 'rtabmap_map')).toBeNull()
  })

  describe('rejection logging', () => {
    beforeEach(() => {
      vi.mocked(log.warn).mockClear()
    })

    it('logs a foreign frame once, however often it repeats', async () => {
      vi.resetModules()
      const fresh = await import('./cloudDecode')
      const warn = (await import('../logging')).default.warn as ReturnType<typeof vi.fn>
      warn.mockClear()
      const bad = build([1, 2, 3], undefined, { frame_id: 'map' })
      expect(fresh.decodeMap3dCloud(bad, 'rtabmap_map')).toBeNull()
      expect(fresh.decodeMap3dCloud(bad, 'rtabmap_map')).toBeNull()
      expect(warn).toHaveBeenCalledTimes(1)
      expect(String(warn.mock.calls[0][0])).toContain('frame')
    })

    it('logs a byte-length mismatch once', async () => {
      vi.resetModules()
      const fresh = await import('./cloudDecode')
      const warn = (await import('../logging')).default.warn as ReturnType<typeof vi.fn>
      warn.mockClear()
      const bad = build([1, 2, 3, 4, 5, 6], undefined, { count: 3 })
      expect(fresh.decodeMap3dCloud(bad, 'rtabmap_map')).toBeNull()
      expect(fresh.decodeMap3dCloud(bad, 'rtabmap_map')).toBeNull()
      expect(warn).toHaveBeenCalledTimes(1)
      expect(String(warn.mock.calls[0][0])).toContain('do not match count')
    })
  })
})
