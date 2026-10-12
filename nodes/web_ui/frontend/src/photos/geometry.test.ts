import { describe, expect, it } from 'vitest'
import type { Photo, PhotoView } from './api'
import { ageCutoff, cameraColor, DEFAULT_HFOV_RAD, newestPhotos, photoWedge, WEDGE_LENGTH_M } from './geometry'

function view(over: Partial<PhotoView> = {}): PhotoView {
  return { x: 1, y: 2, z: 0.3, yaw: 0, pitch: 0.1, hfov: Math.PI / 2, source: 'camera_mount', ...over }
}

function photo(id: string, over: Partial<Photo> = {}): Photo {
  return { id, ts: 100, camera: 'head', tool: 't', name: '', note: '', poiIds: [], x: 1, y: 2, view: view(), bytes: 1, width: 1, height: 1, ...over }
}

describe('photoWedge', () => {
  it('is half the original 0.35 m long, apex to each edge', () => {
    expect(WEDGE_LENGTH_M).toBeCloseTo(0.175)
    const w = photoWedge(photo('a'))!
    for (const edge of [w.vertices[1], w.vertices[2]]) {
      expect(Math.hypot(edge[0] - w.vertices[0][0], edge[1] - w.vertices[0][1])).toBeCloseTo(WEDGE_LENGTH_M)
    }
  })

  it('puts the apex at the view and the two corners at +-hfov/2 around the yaw, in map coordinates', () => {
    const w = photoWedge(photo('a'))!
    expect(w.dashed).toBe(false)
    expect(w.vertices).toHaveLength(3)
    expect(w.vertices[0]).toEqual([1, 2])
    const half = Math.PI / 4
    expect(w.vertices[1][0]).toBeCloseTo(1 + WEDGE_LENGTH_M * Math.cos(half))
    expect(w.vertices[1][1]).toBeCloseTo(2 + WEDGE_LENGTH_M * Math.sin(half))
    expect(w.vertices[2][0]).toBeCloseTo(1 + WEDGE_LENGTH_M * Math.cos(-half))
    expect(w.vertices[2][1]).toBeCloseTo(2 + WEDGE_LENGTH_M * Math.sin(-half))
  })

  it('rotates the wedge with the yaw', () => {
    const w = photoWedge(photo('a', { view: view({ yaw: Math.PI / 2, hfov: Math.PI / 2 }) }))!
    // centre line points to +y: both corners are above the apex
    expect(w.vertices[1][1]).toBeGreaterThan(2)
    expect(w.vertices[2][1]).toBeGreaterThan(2)
    expect(w.vertices[1][0]).toBeCloseTo(1 - WEDGE_LENGTH_M * Math.sin(Math.PI / 4))
    expect(w.vertices[2][0]).toBeCloseTo(1 + WEDGE_LENGTH_M * Math.sin(Math.PI / 4))
  })

  it('uses the display-only default width and a dashed flag when hfov is unknown', () => {
    const w = photoWedge(photo('a', { view: view({ hfov: null }) }))!
    expect(w.dashed).toBe(true)
    const half = DEFAULT_HFOV_RAD / 2
    expect(w.vertices[1][0]).toBeCloseTo(1 + WEDGE_LENGTH_M * Math.cos(half))
    expect(DEFAULT_HFOV_RAD).toBeCloseTo((60 * Math.PI) / 180)
  })

  it('produces nothing for an unlocated photo', () => {
    expect(photoWedge(photo('a', { view: null, x: null, y: null }))).toBeNull()
  })
})

describe('layer helpers', () => {
  it('keeps the newest photos up to the limit', () => {
    const photos = [photo('a', { ts: 1 }), photo('b', { ts: 3 }), photo('c', { ts: 2 })]
    expect(newestPhotos(photos, 2).map((p) => p.id)).toEqual(['b', 'c'])
  })

  it('maps the age filter to a cutoff time', () => {
    expect(ageCutoff('24h', 100000)).toBe(100000 - 86400)
    expect(ageCutoff('7d', 1_000_000)).toBe(1_000_000 - 7 * 86400)
    expect(ageCutoff('all', 1_000_000)).toBeNull()
  })

  it('gives the same colour to the same camera and different colours to different cameras', () => {
    expect(cameraColor('head')).toBe(cameraColor('head'))
    expect(cameraColor('head')).not.toBe(cameraColor('gripper'))
  })
})
