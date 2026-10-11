import { describe, expect, it, vi } from 'vitest'
import { deletePhotos, fetchPhotos, photoImageUrl, photosUrl, photoThumbUrl, THUMB_PX } from './api'

function fakeFetch(status: number, body: unknown) {
  return vi.fn(async () => ({ ok: status < 400, status, json: async () => body }) as Response)
}

const VIEW = { x: 1, y: 2, z: 0.3, yaw: 0.5, pitch: 0.1, hfov: 1.2, source: 'camera_mount' }
const PHOTO = {
  id: 'b',
  kind: 'photo',
  x: 1,
  y: 2,
  name: 'mug',
  note: 'n',
  status: 'open',
  created_by: 'agent',
  created_at: 1760176800,
  updated_at: 0,
  poi_ids: ['p'],
  photo: { file: 'b.jpg', bytes: 2048, width: 640, height: 480, camera: 'head', tool: 'get_camera_image', mcp_session: 's', view: VIEW },
}

describe('photosUrl', () => {
  it('has only the limit without filters', () => {
    expect(photosUrl(null, 8, null)).toBe('/api/photos?limit=8')
  })

  it('adds the POI filter (URL-encoded) and the cursor', () => {
    expect(photosUrl('poi 1/x', 5, '1760.5:abc')).toBe('/api/photos?limit=5&poi_id=poi%201%2Fx&before=1760.5%3Aabc')
  })
})

describe('image urls', () => {
  it('point at the photos jpg route, thumbnails are downscaled', () => {
    expect(photoImageUrl('abc')).toBe('/api/photos/abc.jpg')
    expect(photoThumbUrl('abc')).toBe(`/api/photos/abc.jpg?max_px=${THUMB_PX}`)
    expect(photoThumbUrl('a b', 96)).toBe('/api/photos/a%20b.jpg?max_px=96')
  })
})

describe('fetchPhotos', () => {
  it('parses photo POIs including the view and null x/y', async () => {
    const unlocated = { ...PHOTO, id: 'a', x: null, y: null, photo: { ...PHOTO.photo, view: null }, poi_ids: [] }
    const f = fakeFetch(200, { photos: [PHOTO, unlocated, { nope: 1 }], next_before: '5:a', total_matching: 7 })
    const out = await fetchPhotos('p', 10, null, f as unknown as typeof fetch)
    expect(out.nextBefore).toBe('5:a')
    expect(out.total).toBe(7)
    expect(out.photos.map((p) => p.id)).toEqual(['b', 'a'])
    expect(out.photos[0]).toMatchObject({ camera: 'head', tool: 'get_camera_image', name: 'mug', note: 'n', poiIds: ['p'], bytes: 2048, ts: 1760176800 })
    expect(out.photos[0].view).toEqual(VIEW)
    expect(out.photos[1]).toMatchObject({ x: null, y: null, view: null, poiIds: [] })
    expect((f.mock.calls[0] as unknown as [string])[0]).toBe('/api/photos?limit=10&poi_id=p')
  })

  it('reports the last page as a null cursor', async () => {
    const out = await fetchPhotos(null, 10, null, fakeFetch(200, { photos: [], next_before: null, total_matching: 0 }) as unknown as typeof fetch)
    expect(out).toEqual({ photos: [], nextBefore: null, total: 0 })
  })

  it('rejects with a readable reason on HTTP errors, bad bodies and network failures', async () => {
    await expect(fetchPhotos(null, 5, null, fakeFetch(404, {}) as unknown as typeof fetch)).rejects.toThrow('HTTP 404')
    await expect(fetchPhotos(null, 5, null, fakeFetch(200, { nope: 1 }) as unknown as typeof fetch)).rejects.toThrow('unexpected response')
    const boom = vi.fn(async () => {
      throw new Error('offline')
    })
    await expect(fetchPhotos(null, 5, null, boom as unknown as typeof fetch)).rejects.toThrow('offline')
  })
})

describe('deletePhotos', () => {
  it('posts the delete_photos op with the selector and maps a real delete', async () => {
    const f = fakeFetch(200, { ok: true, message: 'ok', poi: { removed: 2, bytes_freed: 3000, missing: ['x'] } })
    const out = await deletePhotos('t 1', { ids: ['a', 'b'] }, f as unknown as typeof fetch)
    const [url, init] = f.mock.calls[0] as unknown as [string, RequestInit]
    expect(url).toBe('/api/poi?tab=t%201')
    expect(JSON.parse(String(init.body))).toEqual({ op: 'delete_photos', selector: { ids: ['a', 'b'] } })
    expect(out).toEqual({ ok: true, message: 'ok', count: 2, bytes: 3000 })
  })

  it('maps a dry run to would_remove / bytes', async () => {
    const f = fakeFetch(200, { ok: true, message: 'would delete', poi: { would_remove: 4, bytes: 5_000_000 } })
    const out = await deletePhotos('t', { older_than_s: 86400, dry_run: true }, f as unknown as typeof fetch)
    expect(JSON.parse(String((f.mock.calls[0] as unknown as [string, RequestInit])[1].body)).selector).toEqual({ older_than_s: 86400, dry_run: true })
    expect(out).toEqual({ ok: true, message: 'would delete', count: 4, bytes: 5_000_000 })
  })

  it('returns the failure message', async () => {
    const out = await deletePhotos('t', { keep_newest: 3 }, fakeFetch(422, { ok: false, message: 'bad selector', poi: null }) as unknown as typeof fetch)
    expect(out).toMatchObject({ ok: false, message: 'bad selector', count: 0, bytes: 0 })
  })
})

describe('since filter', () => {
  it('adds since to the photos URL only when given', () => {
    expect(photosUrl(null, 1000, null, 1234)).toBe('/api/photos?limit=1000&since=1234')
    expect(photosUrl(null, 1000, null)).toBe('/api/photos?limit=1000')
  })
})
